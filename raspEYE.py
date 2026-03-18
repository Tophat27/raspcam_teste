#!/usr/bin/env python3
"""
raspEYE.py — Captura TCP + Correção Fisheye OV5647 + Upload Google Drive
Acionado pelo radar MR60BHA2 via endpoint HTTP /trigger
"""

import os
import sys

try:
    import numpy as _np
except Exception:
    pass
else:
    _v = getattr(_np, "__version__", "0").split(".")[:2]
    try:
        if int(_v[0]) >= 2:
            print("NumPy 2.x detectado. Execute: pip install \"numpy<2\"")
            sys.exit(1)
    except (ValueError, IndexError):
        pass

os.environ.setdefault("DISPLAY", "")

import cv2
import numpy as np
import time
import json
import threading
import traceback
from datetime import datetime
from flask import Flask, request, jsonify
from google.oauth2 import service_account
from googleapiclient.discovery import build
from googleapiclient.http import MediaFileUpload

# ============================================================
# CONFIGURAÇÕES
# ============================================================
SCRIPT_DIR              = os.path.dirname(os.path.abspath(__file__))
STREAM_URL              = "tcp://localhost:8888"
FRAME_SAVE_INTERVAL     = 5
UPLOAD_INTERVAL_SECONDS = 60
LOCAL_FRAME_DIR         = os.path.join(SCRIPT_DIR, "frames_temp")
SERVICE_ACCOUNT_FILE    = os.path.join(SCRIPT_DIR, "service-account.json")
MAX_UPLOAD_RETRIES      = 3
RETRY_INFO_FILE         = os.path.join(LOCAL_FRAME_DIR, "upload_failures.json")
DRIVE_ROOT_FOLDER_ID    = "1GMRT-ba9PAFaav8q9UTGjNLf2ZAjMfgm"

MAX_LOCAL_FILES         = 500
MAX_LOCAL_MB            = 200

# Porta do endpoint HTTP que o ESP32 vai chamar
TRIGGER_PORT            = 5001

# Segundos sem sinal do radar antes de parar a captura automaticamente
# (segurança caso o ESP32 pare de enviar)
RADAR_TIMEOUT_S         = 15.0

FISHEYE_ENABLED = True
FISHEYE_CFG = dict(
    fx=220.0, fy=220.0,
    cx=320.0, cy=230.0,
    k1=0.0, k2=0.0, k3=0.0, k4=0.0,
    rotate_180=True,
)


# ============================================================
# ESTADO DO RADAR (compartilhado entre threads)
# ============================================================
class RadarState:
    def __init__(self):
        self._lock           = threading.Lock()
        self.human_detected  = False
        self.last_seen       = 0.0    # timestamp do último POST do radar
        self.num_targets     = 0
        self.breath_rate     = 0.0
        self.heart_rate      = 0.0
        self.distance        = 0.0

    def update(self, human: bool, data: dict):
        with self._lock:
            self.human_detected = human
            self.last_seen      = time.time()
            self.num_targets    = data.get("num_targets", 0)
            vs = data.get("vital_signs_available", False)
            if vs:
                self.breath_rate = data.get("breath_rate", 0.0)
                self.heart_rate  = data.get("heart_rate",  0.0)
                self.distance    = data.get("distance",    0.0)

    @property
    def active(self) -> bool:
        """True se humano detectado E radar ainda enviando dados recentemente."""
        with self._lock:
            timed_out = (time.time() - self.last_seen) > RADAR_TIMEOUT_S
            return self.human_detected and not timed_out


radar = RadarState()


# ============================================================
# ENDPOINT FLASK (recebe dados do ESP32)
# ============================================================
app = Flask(__name__)

@app.route("/trigger", methods=["POST"])
def trigger():
    """
    Recebe o JSON do ESP32 (mesmo formato já usado em sendToAPI).
    Só atualiza o estado — não faz mais nada.
    """
    try:
        payload = request.get_json(force=True, silent=True) or {}
        data    = payload.get("data", {})
        human   = bool(data.get("human_detected", False))
        radar.update(human, data)

        status = "CAPTURANDO" if radar.active else "OCIOSO"
        return jsonify({"ok": True, "status": status}), 200

    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/status", methods=["GET"])
def status():
    """Endpoint de diagnóstico — mostra estado atual."""
    with radar._lock:
        return jsonify({
            "capturing":     radar.active,
            "human":         radar.human_detected,
            "targets":       radar.num_targets,
            "breath_rate":   radar.breath_rate,
            "heart_rate":    radar.heart_rate,
            "distance_cm":   radar.distance,
            "last_radar_s":  round(time.time() - radar.last_seen, 1),
        })


def start_flask():
    app.run(host="0.0.0.0", port=TRIGGER_PORT,
            debug=False, use_reloader=False)


# ============================================================
# CORRETOR FISHEYE
# ============================================================
class FisheyeCorrector:
    def __init__(self, w, h, cfg):
        K = np.array([[cfg["fx"], 0,         cfg["cx"]],
                      [0,         cfg["fy"], cfg["cy"]],
                      [0,         0,         1        ]], dtype=np.float64)
        D = np.array([[cfg["k1"]], [cfg["k2"]],
                      [cfg["k3"]], [cfg["k4"]]], dtype=np.float64)
        nova_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            K, D, (w, h), np.eye(3), balance=0.0)
        self._m1, self._m2 = cv2.fisheye.initUndistortRectifyMap(
            K, D, np.eye(3), nova_K, (w, h), cv2.CV_16SC2)
        self._rotate = cfg.get("rotate_180", False)
        print(f"FisheyeCorrector pronto: {w}x{h} rotate={self._rotate}")

    def correct(self, frame):
        out = cv2.remap(frame, self._m1, self._m2,
                        interpolation=cv2.INTER_LINEAR,
                        borderMode=cv2.BORDER_CONSTANT,
                        borderValue=(0, 0, 0))
        if self._rotate:
            out = cv2.rotate(out, cv2.ROTATE_180)
        return out


# ============================================================
# PROTEÇÃO DE DISCO
# ============================================================
def _local_usage():
    files = [f for f in os.listdir(LOCAL_FRAME_DIR) if f.endswith(".jpg")]
    total_bytes = sum(
        os.path.getsize(os.path.join(LOCAL_FRAME_DIR, f)) for f in files)
    return len(files), total_bytes / (1024 * 1024)


def local_storage_ok():
    n, mb = _local_usage()
    if n >= MAX_LOCAL_FILES:
        print(f"DISCO CHEIO: {n} arquivos (limite {MAX_LOCAL_FILES}). Pausando...")
        return False
    if mb >= MAX_LOCAL_MB:
        print(f"DISCO CHEIO: {mb:.1f} MB (limite {MAX_LOCAL_MB} MB). Pausando...")
        return False
    return True


# ============================================================
# GOOGLE DRIVE
# ============================================================
def authenticate_drive():
    try:
        credentials = service_account.Credentials.from_service_account_file(
            SERVICE_ACCOUNT_FILE,
            scopes=["https://www.googleapis.com/auth/drive"])
        print("Autenticado com service account!")
        return build("drive", "v3", credentials=credentials)
    except Exception as e:
        print(f"Erro na autenticacao: {e}")
        traceback.print_exc()
        return None


drive_service = authenticate_drive()
if not drive_service:
    sys.exit(1)

_folder_cache = {}
_folder_lock  = threading.Lock()


def _get_or_create_folder(name, parent_id):
    query = (f"name='{name}' and '{parent_id}' in parents and "
             f"mimeType='application/vnd.google-apps.folder' and trashed=false")
    results = drive_service.files().list(
        q=query, fields="files(id)", pageSize=1).execute()
    items = results.get("files", [])
    if items:
        return items[0]["id"]
    metadata = {"name": name,
                "mimeType": "application/vnd.google-apps.folder",
                "parents": [parent_id]}
    folder = drive_service.files().create(body=metadata, fields="id").execute()
    print(f"  Pasta criada no Drive: {name}")
    return folder["id"]


def get_drive_folder_for(dt):
    day_key   = dt.strftime("%Y-%m-%d")
    hour_key  = dt.strftime("%H")
    cache_key = f"{day_key}/{hour_key}"
    with _folder_lock:
        if cache_key in _folder_cache:
            return _folder_cache[cache_key]
        day_id  = _get_or_create_folder(day_key,  DRIVE_ROOT_FOLDER_ID)
        hour_id = _get_or_create_folder(hour_key, day_id)
        _folder_cache[cache_key] = hour_id
        return hour_id


# ============================================================
# RETRY INFO
# ============================================================
def _load_retry_info():
    if not os.path.exists(RETRY_INFO_FILE):
        return {}
    try:
        with open(RETRY_INFO_FILE, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {}


def _save_retry_info(info):
    try:
        with open(RETRY_INFO_FILE, "w", encoding="utf-8") as f:
            json.dump(info, f)
    except Exception as e:
        print(f"Nao foi possivel salvar retry info: {e}")


# ============================================================
# THREAD DE UPLOAD
# ============================================================
def upload_frames():
    print(f"Thread de upload iniciada (a cada {UPLOAD_INTERVAL_SECONDS}s)")
    retry_info = _load_retry_info()
    while True:
        time.sleep(UPLOAD_INTERVAL_SECONDS)
        try:
            files_to_upload = sorted(
                f for f in os.listdir(LOCAL_FRAME_DIR) if f.endswith(".jpg"))
            if not files_to_upload:
                continue

            n, mb = _local_usage()
            print(f"Upload: {len(files_to_upload)} frames | {mb:.1f} MB local")

            for filename in files_to_upload:
                file_path = os.path.join(LOCAL_FRAME_DIR, filename)
                if retry_info.get(filename, 0) >= MAX_UPLOAD_RETRIES:
                    try:
                        os.remove(file_path)
                    except Exception:
                        pass
                    retry_info.pop(filename, None)
                    continue
                try:
                    parts = filename.split("_")
                    dt = datetime.strptime(f"{parts[1]}_{parts[2]}", "%Y%m%d_%H%M%S")
                except Exception:
                    dt = datetime.now()
                try:
                    folder_id = get_drive_folder_for(dt)
                    media = MediaFileUpload(
                        file_path, mimetype="image/jpeg", resumable=True)
                    uploaded = drive_service.files().create(
                        body={"name": filename, "parents": [folder_id]},
                        media_body=media, fields="id").execute()
                    os.remove(file_path)
                    retry_info.pop(filename, None)
                    print(f"  OK {filename} -> {dt.strftime('%Y-%m-%d/%H')}")
                except Exception as e:
                    retry_info[filename] = retry_info.get(filename, 0) + 1
                    print(f"  ERRO {filename} ({retry_info[filename]}/{MAX_UPLOAD_RETRIES}): {e}")
                    traceback.print_exc()

            _save_retry_info(retry_info)
        except Exception as e:
            print(f"Erro geral no upload: {e}")
            traceback.print_exc()


# ============================================================
# INICIALIZAÇÃO
# ============================================================
os.makedirs(LOCAL_FRAME_DIR, exist_ok=True)

# Flask em thread separada
threading.Thread(target=start_flask, daemon=True).start()
print(f"Endpoint do radar: http://0.0.0.0:{TRIGGER_PORT}/trigger")
print(f"Status:            http://0.0.0.0:{TRIGGER_PORT}/status")

# Upload em thread separada
threading.Thread(target=upload_frames, daemon=True).start()

# ============================================================
# CAPTURA PRINCIPAL
# ============================================================
print(f"Conectando ao stream TCP: {STREAM_URL}")
cap = cv2.VideoCapture(STREAM_URL)
if not cap.isOpened():
    print("Nao foi possivel abrir o stream TCP.")
    sys.exit(1)

corrector   = None
frame_count = 0

try:
    while True:
        ret, frame = cap.read()

        if not ret:
            print("Stream interrompido, reconectando em 2s...")
            time.sleep(2)
            cap.release()
            cap = cv2.VideoCapture(STREAM_URL)
            corrector = None
            continue

        # Inicializar corretor na primeira vez
        if corrector is None and FISHEYE_ENABLED:
            h, w = frame.shape[:2]
            corrector = FisheyeCorrector(w, h, FISHEYE_CFG)

        frame_count += 1

        # ── SÓ CAPTURA SE O RADAR INDICAR HUMANO ────────────────
        if not radar.active:
            time.sleep(0.05)   # evita busy-loop quando ocioso
            continue

        if frame_count % FRAME_SAVE_INTERVAL != 0:
            continue

        if not local_storage_ok():
            time.sleep(5)
            continue

        frame_final = corrector.correct(frame) if corrector else frame

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        filename  = f"frame_{timestamp}_{frame_count}.jpg"
        file_path = os.path.join(LOCAL_FRAME_DIR, filename)

        cv2.imwrite(file_path, frame_final, [cv2.IMWRITE_JPEG_QUALITY, 95])
        print(f"Salvo: {filename}  "
              f"[targets={radar.num_targets} dist={radar.distance:.0f}cm]")

except KeyboardInterrupt:
    print("\nInterrompido pelo usuario")
finally:
    cap.release()
    print("Sistema finalizado")
