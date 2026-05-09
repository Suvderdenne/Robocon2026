# -*- coding: utf-8 -*-
"""
ABU Robocon 2026 - R2 KFS Recognition System  (ЗАСВАРЛАСАН v2)
==============================================================
Засварууд:
  - ORB + Histogram + SSIM гурвыг нэгтгэсэн (илүү найдвартай)
  - Confidence тооцооны алдаа засав
  - KFS хайрцаг detect сайжруулав (Canny + white HSV)
  - Template DB хоосон байхад сэрэмжлүүлэг харуулна
  - Кодыг цэвэрлэж, тайлбар нэмсэн

Шаардлага:
  pip install opencv-python numpy
"""

import sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")

import cv2
import numpy as np
import os
import json
import time
from datetime import datetime


# ─── ТОХИРГОО ─────────────────────────────────────────────────────────────────
CONFIG = {
    # Хавтасны замууд
    "template_real_dir":  "templates/real",
    "template_fake_dir":  "templates/fake",
    "results_dir":        "results",
    "save_detected_dir":  "results/detected",

    # ORB тохиргоо
    "orb_features":       500,
    "orb_distance_max":   60,    # Бага = илүү нарийн шүүлт
    "orb_match_min":      10,    # ORB-д хэдэн good match хэрэгтэй

    # Histogram тохиргоо
    "hist_bins":          64,
    "hist_min_score":     0.40,  # 0~1: Histogram correlation доод хязгаар

    # SSIM тохиргоо  (cv2.matchTemplate ашиглана)
    "ssim_min_score":     0.30,

    # Эцсийн шийдвэр
    "combo_threshold":    0.40,  # Нийлсэн score-н доод хязгаар

    # KFS цагаан хайрцаг detect
    "white_lower":        [0,  0,  160],
    "white_upper":        [180, 50, 255],
    "min_kfs_area":       1500,
    "kfs_margin_ratio":   0.10,

    # Ногоон блок detect
    "green_lower":        [35, 40, 40],
    "green_upper":        [85, 255, 255],
    "min_block_area":     3000,

    # Камер
    "camera_id":          0,

    # Debug горим
    "debug":              True,

    # Capture горимын дэлгэцийн хэмжээ
    "frame_w":            640,
    "frame_h":            480,
}


# ─── PREPROCESSING ────────────────────────────────────────────────────────────

def preprocess(img_gray, size=128):
    """
    Зургийг стандарт хэмжээнд оруулж binary болгоно.
    Гэрлийн нөхцөлөөс хамааралгүй болгоно.
    """
    img = cv2.resize(img_gray, (size, size))
    img = cv2.GaussianBlur(img, (3, 3), 0)
    img = cv2.adaptiveThreshold(
        img, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        11, 2
    )
    return img


# ─── TEMPLATE DATABASE ────────────────────────────────────────────────────────

def load_templates(config):
    """
    templates/real/ болон templates/fake/ доторх зургуудыг ачаалж
    ORB descriptor + histogram-г тооцно.

    Returns:
        {"real": [...], "fake": [...]}
        Тус бүрийн элемент:
            {"name": str, "proc": np.ndarray, "des": np.ndarray, "hist": np.ndarray}
    """
    orb = cv2.ORB_create(nfeatures=config["orb_features"])
    db  = {"real": [], "fake": []}

    for label in ("real", "fake"):
        folder = config[f"template_{label}_dir"]
        if not os.path.exists(folder):
            print(f"[WARN] '{folder}' not found folder.")
            continue

        files = sorted(
            f for f in os.listdir(folder)
            if f.lower().endswith((".png", ".jpg", ".jpeg"))
        )

        for fname in files:
            path = os.path.join(folder, fname)
            img  = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            if img is None:
                print(f"[WARN] can't read: {path}")
                continue

            proc = preprocess(img)

            # ORB descriptor
            kp, des = cv2.ORB_create(nfeatures=config["orb_features"]).detectAndCompute(proc, None)

            # Histogram (normalized)
            hist = cv2.calcHist([proc], [0], None, [config["hist_bins"]], [0, 256])
            cv2.normalize(hist, hist)

            if des is not None:
                db[label].append({
                    "name": os.path.splitext(fname)[0],
                    "proc": proc,
                    "des":  des,
                    "hist": hist,
                })

    print(f"[Template DB] Real: {len(db['real'])}  Fake: {len(db['fake'])}")
    return db


# ─── MATCHING FUNCTIONS ───────────────────────────────────────────────────────

def _orb_score(des_q, des_t, config):
    """ORB Brute-Force matching → 0.0~1.0 score"""
    bf      = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des_q, des_t)
    good    = [m for m in matches if m.distance < config["orb_distance_max"]]
    # max_possible = orb_features гэхдээ хоёр template-н keypoint-н min
    return min(len(good) / max(config["orb_match_min"], 1), 1.0)


def _hist_score(hist_q, hist_t):
    """Histogram Correlation → 0.0~1.0 score"""
    score = cv2.compareHist(hist_q, hist_t, cv2.HISTCMP_CORREL)
    return max(float(score), 0.0)   # negative үр дүнг 0 болгоно


def _ssim_score(proc_q, proc_t):
    """
    Template Matching (TM_CCOEFF_NORMED) ашиглан SSIM-тэй ойролцоо score тооцно.
    Хоёр зураг ижил хэмжээтэй (128x128) тул шууд harьцуулна.
    """
    res   = cv2.matchTemplate(proc_q, proc_t, cv2.TM_CCOEFF_NORMED)
    score = float(np.max(res))
    return max(score, 0.0)


def match_symbol(query_img, template_db, config):
    """
    Query зургийг template DB-тай ORB + Histogram + SSIM-ийн нийлбэрээр харьцуулна.

    Returns:
        {
            "label":      "REAL" | "FAKE" | "UNKNOWN",
            "name":       str,
            "confidence": 0.0~1.0,
            "orb":        float,
            "hist":       float,
            "ssim":       float,
        }
    """
    orb = cv2.ORB_create(nfeatures=config["orb_features"])

    proc_q = preprocess(query_img)
    kp_q, des_q = orb.detectAndCompute(proc_q, None)

    hist_q = cv2.calcHist([proc_q], [0], None, [config["hist_bins"]], [0, 256])
    cv2.normalize(hist_q, hist_q)

    if des_q is None:
        return _unknown()

    best = {"combo": -1, "label": "UNKNOWN", "name": "", "orb": 0, "hist": 0, "ssim": 0}

    for label in ("real", "fake"):
        for tmpl in template_db[label]:
            try:
                o = _orb_score(des_q, tmpl["des"], config)
                h = _hist_score(hist_q, tmpl["hist"])
                s = _ssim_score(proc_q, tmpl["proc"])

                # Нийлсэн score (жин: ORB 40%, Hist 40%, SSIM 20%)
                combo = 0.40 * o + 0.40 * h + 0.20 * s

                if combo > best["combo"]:
                    best = {
                        "combo": combo,
                        "label": label.upper(),
                        "name":  tmpl["name"],
                        "orb":   round(o, 2),
                        "hist":  round(h, 2),
                        "ssim":  round(s, 2),
                    }
            except Exception as e:
                continue

    if best["combo"] < config["combo_threshold"]:
        return _unknown(best)

    return {
        "label":      best["label"],
        "name":       best["name"],
        "confidence": round(best["combo"], 2),
        "orb":        best["orb"],
        "hist":       best["hist"],
        "ssim":       best["ssim"],
    }


def _unknown(info=None):
    base = {"label": "UNKNOWN", "name": "", "confidence": 0.0, "orb": 0, "hist": 0, "ssim": 0}
    if info:
        base["confidence"] = round(info.get("combo", 0), 2)
    return base


# ─── KFS ХАЙРЦАГ ИЛРҮҮЛЭХ ────────────────────────────────────────────────────

def find_kfs_box(frame, config):
    """
    Frame доторхи цагаан KFS хайрцгийг илрүүлнэ.
    Арга 1: HSV цагаан маск
    Арга 2: Canny edge (нөөц арга)

    Returns:
        (gray_crop, (x, y, w, h)) эсвэл (None, None)
    """
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lo   = np.array(config["white_lower"])
    hi   = np.array(config["white_upper"])
    mask = cv2.inRange(hsv, lo, hi)

    k    = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Хэрэв HSV маск ажиллахгүй бол Canny ашиглана
    if not contours:
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_cnt = None
    best_score = 0

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < config["min_kfs_area"]:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        aspect     = w / float(h) if h > 0 else 0
        if not (0.5 < aspect < 2.0):
            continue
        # Квадратлаг байх тусам score өндөр
        squareness = 1.0 - abs(1.0 - aspect)
        score = area * squareness
        if score > best_score:
            best_score = score
            best_cnt   = cnt

    if best_cnt is None:
        return None, None

    x, y, w, h = cv2.boundingRect(best_cnt)
    m  = int(min(w, h) * config["kfs_margin_ratio"])
    cx, cy = x + m, y + m
    cw, ch = w - 2 * m, h - 2 * m
    if cw < 20 or ch < 20:
        return None, None

    crop = frame[cy:cy + ch, cx:cx + cw]
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    return gray, (x, y, w, h)


# ─── НОГООН БЛОК DETECT ───────────────────────────────────────────────────────

def find_green_blocks(frame, config):
    """Meihua Forest-н ногоон блокуудыг илрүүлнэ."""
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lo   = np.array(config["green_lower"])
    hi   = np.array(config["green_upper"])
    mask = cv2.inRange(hsv, lo, hi)

    k    = np.ones((7, 7), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blocks = [c for c in contours if cv2.contourArea(c) > config["min_block_area"]]
    return blocks, mask


# ─── ДЭЛГЭЦЭН ДЭЭР ХАРУУЛАХ ТУСЛАХ ФУНКЦУУД ─────────────────────────────────

def _draw_result(display, current_result, kfs_bbox, blocks, config):
    """Frame дээр бүх мэдээллийг зурна."""

    # Ногоон блок тойм
    if config["debug"]:
        for cnt in blocks:
            cv2.drawContours(display, [cnt], -1, (0, 200, 0), 1)

    # KFS хайрцаг тойм
    if kfs_bbox is not None:
        x, y, w, h = kfs_bbox
        color = (0, 200, 200)  # default: шар
        if current_result:
            if current_result["label"] == "REAL":
                color = (0, 230, 0)     # ногоон
            elif current_result["label"] == "FAKE":
                color = (0, 0, 220)     # улаан
        cv2.rectangle(display, (x, y), (x + w, y + h), color, 2)

    # Үр дүн хэсэг (доод тал)
    if current_result:
        label = current_result["label"]
        conf  = current_result["confidence"]

        if label == "REAL":
            txt_col = (0, 220, 0)
            bg_col  = (0, 50, 0)
            action  = "TSUGLUUL"   # latin fallback
        elif label == "FAKE":
            txt_col = (0, 80, 220)
            bg_col  = (50, 0, 0)
            action  = "ORHI!"
        else:
            txt_col = (160, 160, 160)
            bg_col  = (30, 30, 30)
            action  = "TODORKHYIGUI"

        cv2.rectangle(display, (0, 390), (640, 480), bg_col, -1)
        cv2.putText(display,
                    f"{label}  {action}",
                    (10, 425), cv2.FONT_HERSHEY_SIMPLEX, 1.0, txt_col, 2)

        detail = (
            f"conf:{conf:.0%}  "
            f"orb:{current_result.get('orb', 0):.2f}  "
            f"hist:{current_result.get('hist', 0):.2f}  "
            f"ssim:{current_result.get('ssim', 0):.2f}  "
            f"char:{current_result['name']}"
        )
        cv2.putText(display, detail,
                    (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.45, txt_col, 1)
    else:
        msg = "KFS box harahdakhgui..." if kfs_bbox is None else "Template DB khooson"
        cv2.putText(display, msg, (10, 460),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (120, 120, 120), 1)

    cv2.putText(display, "S=save  R=JSON  Q=exit",
                (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)


# ─── RESULT LOGGER ────────────────────────────────────────────────────────────

class ResultLogger:
    def __init__(self, config):
        os.makedirs(config["results_dir"],      exist_ok=True)
        os.makedirs(config["save_detected_dir"], exist_ok=True)
        self.config = config
        self.log    = []

    def record(self, block_id, result, img_crop):
        entry = {
            "timestamp":  datetime.now().isoformat(),
            "block_id":   block_id,
            "label":      result["label"],
            "char_name":  result["name"],
            "confidence": result["confidence"],
            "orb":        result.get("orb", 0),
            "hist":       result.get("hist", 0),
            "ssim":       result.get("ssim", 0),
        }
        self.log.append(entry)
        print(
            f"[Block {block_id:02d}] {result['label']:7s} | "
            f"char={result['name']:12s} | conf={result['confidence']:.0%} | "
            f"orb={result.get('orb',0):.2f} hist={result.get('hist',0):.2f} ssim={result.get('ssim',0):.2f}"
        )
        if img_crop is not None:
            fname = (
                f"{self.config['save_detected_dir']}/"
                f"block{block_id:02d}_{result['label'].lower()}_{int(time.time())}.png"
            )
            cv2.imwrite(fname, img_crop)

    def save_json(self):
        path = (
            f"{self.config['results_dir']}/"
            f"scan_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        )
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.log, f, ensure_ascii=False, indent=2)
        print(f"\n[Saved] {path}")
        return path

    def summary(self):
        r = sum(1 for e in self.log if e["label"] == "REAL")
        f = sum(1 for e in self.log if e["label"] == "FAKE")
        u = sum(1 for e in self.log if e["label"] == "UNKNOWN")
        print(f"\n{'='*45}")
        print(f"  REAL    : {r}  (collect)")
        print(f"  FAKE    : {f}  (avoid - violation!)")
        print(f"  UNKNOWN : {u}  (re-check)")
        print(f"{'='*45}")


# ─── TEMPLATE ЦУГЛУУЛАХ ГОРИМ ────────────────────────────────────────────────

def capture_templates_mode(config):
    """
    Webcam дээр KFS харуулж:
      'r' → Real template хадгалах
      'f' → Fake template хадгалах
      'q' → Гарах

    Заавар: python r2_kfs_recognition.py --capture-templates
    """
    print("\n=== TEMPLATE CAPTURE MODE ===")
    print("'r' = Real KFS save  |  'f' = Fake KFS save  |  'q' = quit\n")

    os.makedirs(config["template_real_dir"], exist_ok=True)
    os.makedirs(config["template_fake_dir"], exist_ok=True)

    real_count = len(os.listdir(config["template_real_dir"]))
    fake_count = len(os.listdir(config["template_fake_dir"]))

    cap = cv2.VideoCapture(config["camera_id"])
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config["frame_w"])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config["frame_h"])

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        frame = cv2.resize(frame, (config["frame_w"], config["frame_h"]))

        kfs_gray, bbox = find_kfs_box(frame, config)
        display = frame.copy()

        if bbox is not None:
            x, y, w, h = bbox
            cv2.rectangle(display, (x, y), (x + w, y + h), (0, 255, 200), 2)
            cv2.putText(display, "KFS found - 'r'=Real  'f'=Fake",
                        (x, max(y - 8, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 200), 1)
        else:
            cv2.putText(display, "No KFS detected - show KFS to camera",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 80, 220), 2)

        cv2.putText(display, f"Real:{real_count}  Fake:{fake_count}",
                    (10, config["frame_h"] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imshow("Template Capture", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("r") and kfs_gray is not None:
            real_count += 1
            path = f"{config['template_real_dir']}/real_{real_count:02d}.png"
            cv2.imwrite(path, kfs_gray)
            print(f"[Saved] {path}")
        elif key == ord("f") and kfs_gray is not None:
            fake_count += 1
            path = f"{config['template_fake_dir']}/fake_{fake_count:02d}.png"
            cv2.imwrite(path, kfs_gray)
            print(f"[Saved] {path}")

    cap.release()
    cv2.destroyAllWindows()
    print(f"\nTemplate count - Real: {real_count}  Fake: {fake_count}")


# ─── ҮНДСЭН RECOGNITION PIPELINE ─────────────────────────────────────────────

def run_recognition(config):
    """
    Real-time KFS recognition.

    Товчлуурууд:
      S      → одоогийн үр дүн хадгалах
      R      → JSON хадгалах
      Q/ESC  → гарах
    """
    print("\n=== RECOGNITION MODE ===")
    print("'S'=save frame  'R'=save JSON  'Q'=quit\n")

    template_db = load_templates(config)
    logger      = ResultLogger(config)
    no_templates = not template_db["real"] and not template_db["fake"]

    if no_templates:
        print("⚠  Template DB empty!")
        print("   Run first:  python r2_kfs_recognition.py --capture-templates\n")

    cap = cv2.VideoCapture(config["camera_id"])
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config["frame_w"])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config["frame_h"])

    block_counter    = 0
    last_result      = None
    last_match_time  = 0
    MATCH_INTERVAL   = 0.30   # seconds - тооцооллын давтамж

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[ERROR] Camera read failed.")
            break

        frame = cv2.resize(frame, (config["frame_w"], config["frame_h"]))

        # 1. Ногоон блокууд
        blocks, green_mask = find_green_blocks(frame, config)

        # 2. KFS хайрцаг
        kfs_gray, kfs_bbox = find_kfs_box(frame, config)

        # 3. Template matching (0.3s тутамд)
        current_result = None
        if kfs_gray is not None and not no_templates:
            now = time.time()
            if now - last_match_time > MATCH_INTERVAL:
                last_result     = match_symbol(kfs_gray, template_db, config)
                last_match_time = now
            current_result = last_result

        # 4. Дэлгэц
        display = frame.copy()
        _draw_result(display, current_result, kfs_bbox, blocks, config)

        cv2.imshow("R2 KFS Recognition", display)
        if config["debug"]:
            cv2.imshow("Green mask", green_mask)

        # 5. Товчлуурууд
        key = cv2.waitKey(1) & 0xFF

        if key in (ord("q"), 27):      # Q эсвэл ESC
            break

        elif key == ord("s"):
            if current_result and current_result["label"] != "UNKNOWN" and kfs_gray is not None:
                block_counter += 1
                logger.record(block_counter, current_result, kfs_gray)
            else:
                print("[Skip] No valid result to save.")

        elif key == ord("r"):
            if logger.log:
                logger.save_json()
                logger.summary()
            else:
                print("[Skip] No results logged yet.")

    cap.release()
    cv2.destroyAllWindows()

    # Автоматаар хадгалах
    if logger.log:
        logger.save_json()
        logger.summary()


# ─── ENTRY POINT ──────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("=" * 50)
    print("ABU Robocon 2026 - R2 KFS Recognition v2")
    print("=" * 50)

    if "--capture-templates" in sys.argv or "-c" in sys.argv:
        capture_templates_mode(CONFIG)
    else:
        run_recognition(CONFIG)


# ─── ХЭРЭГЛЭХ ЗААВАР ──────────────────────────────────────────────────────────
"""
АЛХАМ 1 - Хавтас бэлдэх
  mkdir -p kfs_project/templates/real
  mkdir -p kfs_project/templates/fake
  mkdir -p kfs_project/results/detected
  cp r2_kfs_recognition.py kfs_project/
  cd kfs_project

АЛХАМ 2 - Template цуглуулах
  python r2_kfs_recognition.py --capture-templates
  → Rulebook-н Real KFS-г камерт харуулаад 'r' дарна (15 тэмдэгт)
  → Fake KFS-г харуулаад 'f' дарна (15 тэмдэгт)
  ЗӨВЛӨГӨӨ: Нэг тэмдэгтийн 3-5 зураг авбал accuracy сайжирна

АЛХАМ 3 - Тест
  python r2_kfs_recognition.py
  → 'S' дарж тус бүрийн блокийг бүртгэнэ
  → 'R' дарж JSON хадгалана

АЛХАМ 4 - Arduino serial холболт
  import serial
  ser = serial.Serial('/dev/ttyUSB0', 9600)
  # Шийдвэрийн дараа:
  if current_result["label"] == "REAL":
      ser.write(b'COLLECT\n')
  elif current_result["label"] == "FAKE":
      ser.write(b'AVOID\n')

АЛХАМ 5 - Raspberry Pi 5
  sudo apt install python3-opencv python3-numpy
  python3 r2_kfs_recognition.py
  # CONFIG["camera_id"] = 0 хэвээр үлдэнэ
"""