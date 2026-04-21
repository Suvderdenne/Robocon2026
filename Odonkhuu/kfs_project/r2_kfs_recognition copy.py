"""
ABU Robocon 2026 - R2 Kung-Fu Scroll (KFS) Recognition System
=============================================================
Зорилго:
  - Real KFS (Oracle Bone Character) vs Fake KFS (Random Pattern) ялгах
  - Laptop webcam дээр туршиж, дараа Pi5+IMX219 руу шилжүүлэх

Шаардлага:
  pip install opencv-python numpy

Хавтас бүтэц:
  kfs_project/
  ├── r2_kfs_recognition.py   ← энэ файл
  ├── templates/
  │   ├── real/               ← rulebook-н Real KFS зургууд (real_01.png ... real_15.png)
  │   └── fake/               ← rulebook-н Fake KFS зургууд (fake_01.png ... fake_15.png)
  └── results/                ← хадгалсан үр дүнгүүд
"""

# -*- coding: utf-8 -*-# -*- coding: utf-8 -*-
import sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import cv2
import numpy as np
import os
import json
import time
from datetime import datetime

# ─── ТОХИРГОО ──────────────────────────────────────────────────────────────────
MONGOLIAN_TO_LATIN = {
    "а": "a",   "б": "b",   "в": "v",   "г": "g",   "д": "d",
    "е": "e",   "ё": "yo",  "ж": "j",   "з": "z",   "и": "i",
    "й": "i",   "к": "k",   "л": "l",   "м": "m",   "н": "n",
    "о": "o",   "ө": "u",   "п": "p",   "р": "r",   "с": "s",
    "т": "t",   "у": "u",   "ү": "u",   "ф": "f",   "х": "h",
    "ц": "ts",  "ч": "ch",  "ш": "sh",  "щ": "sh",
    "ъ": "",    "ь": "",    "ы": "y",
    "э": "e",   "ю": "yu",  "я": "ya",
}

def transliterate(text: str) -> str:
    result = ""
    for char in text.lower():
        result += MONGOLIAN_TO_LATIN.get(char, char)
    return result

CONFIG = {
    # Хавтасны замууд
    "template_real_dir": "templates/real",
    "template_fake_dir": "templates/fake",
    "results_dir": "results",
    "save_detected_dir": "results/detected",

    # ORB matching тохиргоо
    "orb_features": 500,         # ORB keypoint тоо
    "match_threshold": 15,       # хэдэн match байвал "танигдсан" гэж үзэх
    "min_confidence": 0.4,       # confidence score доод хязгаар (0–1)

    # Ногоон блок detect (HSV)
    "green_lower": [35, 40, 40],
    "green_upper": [85, 255, 255],
    "min_block_area": 3000,      # блокийн хамгийн бага талбай (px²)

    # Цагаан KFS хайрцаг detect (HSV)
    "white_lower": [0, 0, 180],
    "white_upper": [180, 40, 255],
    "min_kfs_area": 500,

    # Webcam индекс
    "camera_id": 0,

    # Debug горим (True = mask, contour харагдана)
    "debug": True,
}


# ─── TEMPLATE DATABASE АЧААЛАХ ────────────────────────────────────────────────

def load_templates(config):
    """
    templates/real/ болон templates/fake/ доторх зургуудыг ачаалж
    ORB descriptor-г тооцно.

    Буцаана: {"real": [...], "fake": [...]}
    Тус бүрийн элемент: {"name": str, "img": np.array, "des": np.array}
    """
    orb = cv2.ORB_create(nfeatures=config["orb_features"])
    db = {"real": [], "fake": []}

    for label in ["real", "fake"]:
        folder = config[f"template_{label}_dir"]
        if not os.path.exists(folder):
            print(f"[АНХААРУУЛГА] {folder} хавтас олдсонгүй. Эхлээд template хадгална уу.")
            continue

        files = [f for f in os.listdir(folder) if f.lower().endswith((".png", ".jpg", ".jpeg"))]
        for fname in sorted(files):
            img = cv2.imread(os.path.join(folder, fname), cv2.IMREAD_GRAYSCALE)
            if img is None:
                continue
            img = preprocess_symbol(img)
            kp, des = orb.detectAndCompute(img, None)
            if des is not None:
                db[label].append({
                    "name": os.path.splitext(fname)[0],
                    "img": img,
                    "des": des,
                })

    print(f"[Template DB] Real: {len(db['real'])} | Fake: {len(db['fake'])}")
    return db


# ─── IMAGE PREPROCESSING ─────────────────────────────────────────────────────

def preprocess_symbol(img_gray, size=128):
    """
    Тэмдэгтийн зургийг стандарт хэмжээнд оруулж, threshold хийнэ.
    Энэ нь гэрлийн өөрчлөлтөнд тэсвэртэй болгоно.
    """
    # Resize
    img = cv2.resize(img_gray, (size, size))
    # Gaussian blur (шуугиан арилгана)
    img = cv2.GaussianBlur(img, (3, 3), 0)
    # Adaptive threshold (тэмдэгтийг цагаан дээр хар болгоно)
    img = cv2.adaptiveThreshold(
        img, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        11, 2
    )
    return img


# ─── ORB MATCHING ─────────────────────────────────────────────────────────────

def match_symbol(query_img, template_db, config):
    """
    Оруулсан зургийг template DB-тай харьцуулна.

    Буцаана:
        {
            "label":      "REAL" | "FAKE" | "UNKNOWN",
            "name":       template нэр (жишээ: "real_03"),
            "confidence": 0.0–1.0,
            "matches":    match тоо,
        }
    """
    orb = cv2.ORB_create(nfeatures=config["orb_features"])
    bf  = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    query_proc = preprocess_symbol(query_img)
    kp_q, des_q = orb.detectAndCompute(query_proc, None)

    if des_q is None:
        return {"label": "UNKNOWN", "name": "", "confidence": 0.0, "matches": 0}

    best = {"score": 0, "label": "UNKNOWN", "name": "", "matches": 0}

    for label in ["real", "fake"]:
        for tmpl in template_db[label]:
            try:
                matches = bf.match(des_q, tmpl["des"])
                matches = sorted(matches, key=lambda m: m.distance)
                # Зөвхөн сайн match авна (distance < 60)
                good = [m for m in matches if m.distance < 60]
                score = len(good)
                if score > best["score"]:
                    best = {
                        "score": score,
                        "label": label.upper(),
                        "name": tmpl["name"],
                        "matches": score,
                    }
            except Exception:
                continue

    # Confidence тооцох
    max_possible = config["orb_features"]
    confidence = min(best["score"] / max(config["match_threshold"], 1), 1.0)

    if best["score"] < config["match_threshold"]:
        return {"label": "UNKNOWN", "name": "", "confidence": confidence, "matches": best["score"]}

    return {
        "label":      best["label"],
        "name":       best["name"],
        "confidence": round(confidence, 2),
        "matches":    best["matches"],
    }


# ─── KFS ХАЙРЦАГ ОЛОХ (ЦАГААН ТЭГШ ӨНЦӨГТ) ──────────────────────────────────

def find_kfs_box(frame, config):
    """
    Frame дотроос цагаан KFS хайрцаг (350x350mm)-г олно.

    Буцаана: crop хийсэн зураг, эсвэл None
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lo  = np.array(config["white_lower"])
    hi  = np.array(config["white_upper"])
    mask = cv2.inRange(hsv, lo, hi)

    kernel = np.ones((5, 5), np.uint8)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, None

    # Хамгийн том дөрвөлжин хэлбэртэй contour
    for cnt in sorted(contours, key=cv2.contourArea, reverse=True):
        area = cv2.contourArea(cnt)
        if area < config["min_kfs_area"]:
            break
        x, y, w, h = cv2.boundingRect(cnt)
        aspect = w / float(h)
        if 0.6 < aspect < 1.6:  # ойролцоогоор квадрат
            # Дотрын тэмдэгт хэсгийг авна (10% margin)
            margin = int(min(w, h) * 0.12)
            cx, cy = x + margin, y + margin
            cw, ch = w - 2 * margin, h - 2 * margin
            if cw > 20 and ch > 20:
                crop = frame[cy:cy + ch, cx:cx + cw]
                gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
                return gray, (x, y, w, h)

    return None, None


# ─── НОГООН БЛОК DETECT ───────────────────────────────────────────────────────

def find_green_blocks(frame, config):
    """
    Meihua Forest-н ногоон блокуудыг олно.
    Буцаана: contour жагсаалт
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lo  = np.array(config["green_lower"])
    hi  = np.array(config["green_upper"])
    mask = cv2.inRange(hsv, lo, hi)

    kernel = np.ones((7, 7), np.uint8)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blocks = [c for c in contours if cv2.contourArea(c) > config["min_block_area"]]
    return blocks, mask


# ─── ҮР ДҮН ХАДГАЛАХ ─────────────────────────────────────────────────────────

class ResultLogger:
    def __init__(self, config):
        self.results_dir      = config["results_dir"]
        self.save_detected    = config["save_detected_dir"]
        os.makedirs(self.results_dir,   exist_ok=True)
        os.makedirs(self.save_detected, exist_ok=True)
        self.log = []

    def record(self, block_id, result, frame_crop):
        """Нэг блокийн үр дүнг хадгална"""
        entry = {
            "timestamp":  datetime.now().isoformat(),
            "block_id":   block_id,
            "label":      result["label"],
            "char_name":  result["name"],
            "confidence": result["confidence"],
            "matches":    result["matches"],
        }
        self.log.append(entry)
        print(
            f"[Block {block_id:02d}] {result['label']:7s} | "
            f"char={result['name']:10s} | conf={result['confidence']:.2f} | "
            f"matches={result['matches']}"
        )

        # Зургийг хадгална
        if frame_crop is not None:
            fname = f"{self.save_detected}/block{block_id:02d}_{result['label'].lower()}_{int(time.time())}.png"
            cv2.imwrite(fname, frame_crop)

    def save_json(self):
        """Бүх үр дүнг JSON файлд хадгална"""
        path = f"{self.results_dir}/scan_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.log, f, ensure_ascii=False, indent=2)
        print(f"\n[Хадгалсан] {path}")
        return path

    def summary(self):
        real_count = sum(1 for e in self.log if e["label"] == "REAL")
        fake_count = sum(1 for e in self.log if e["label"] == "FAKE")
        unkn_count = sum(1 for e in self.log if e["label"] == "UNKNOWN")
        print(f"\n{'='*40}")
        print(f"ДҮНГИЙН ХУРААНГУЙ")
        print(f"  REAL    : {real_count} блок  ← цуглуулах")
        print(f"  FAKE    : {fake_count} блок  ← орхих (violation!)")
        print(f"  UNKNOWN : {unkn_count} блок  ← дахин шалгах")
        print(f"{'='*40}")


# ─── TEMPLATE ЦУГЛУУЛАХ ГОРИМ ────────────────────────────────────────────────

def capture_templates_mode(config):
    """
    Тусгай горим: webcam дээр KFS харуулж 'S' дарахад template хадгална.
    Эхлээд rulebook-н зургийн дагуу 15 real + 15 fake template цуглуулна.

    Ашиглах заавар:
        python r2_kfs_recognition.py --capture-templates
    """
    print("\n=== TEMPLATE ЦУГЛУУЛАХ ГОРИМ ===")
    print("'r' → Real KFS хадгална")
    print("'f' → Fake KFS хадгална")
    print("'q' → Гарна\n")

    os.makedirs(f"{config['template_real_dir']}", exist_ok=True)
    os.makedirs(f"{config['template_fake_dir']}", exist_ok=True)

    cap = cv2.VideoCapture(config["camera_id"])
    real_count = len(os.listdir(config["template_real_dir"]))
    fake_count = len(os.listdir(config["template_fake_dir"]))

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        frame = cv2.resize(frame, (640, 480))

        # KFS хайрцаг олно
        kfs_gray, bbox = find_kfs_box(frame, config)

        display = frame.copy()
        if bbox is not None:
            x, y, w, h = bbox
            cv2.rectangle(display, (x, y), (x + w, y + h), (0, 255, 255), 2)
            cv2.putText(display, "KFS FOUND — 'r'=Real  'f'=Fake",
                        (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        else:
            cv2.putText(display, transliterate("KFS олдсонгүй — KFS-г камерт харуул"),
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        cv2.putText(display, f"Real:{real_count}  Fake:{fake_count}",
                    (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imshow("Template Capture", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r') and kfs_gray is not None:
            real_count += 1
            path = f"{config['template_real_dir']}/real_{real_count:02d}.png"
            cv2.imwrite(path, kfs_gray)
            print(f"[ХАДГАЛСАН] {path}")
        elif key == ord('f') and kfs_gray is not None:
            fake_count += 1
            path = f"{config['template_fake_dir']}/fake_{fake_count:02d}.png"
            cv2.imwrite(path, kfs_gray)
            print(f"[ХАДГАЛСАН] {path}")

    cap.release()
    cv2.destroyAllWindows()
    print(f"\nДүн: Real={real_count}  Fake={fake_count}")


# ─── ҮНДСЭН REAL-TIME PIPELINE ──────────────────────────────────────────────

def run_recognition(config):
    """
    Үндсэн recognition горим.
    Webcam frame бүрт KFS олж, Real/Fake гэдгийг шийдэж дэлгэцэнд харуулна.

    Товчлуурууд:
        S       → одоогийн frame-н үр дүнг хадгална
        R       → бүх үр дүнг JSON хадгална
        Q/ESC   → гарна
    """
    print("\n=== RECOGNITION ГОРИМ ===")
    print("'S' → Frame хадгална | 'R' → JSON хадгална | 'Q' → Гарна\n")

    # Template ачаалах
    template_db = load_templates(config)
    logger      = ResultLogger(config)

    # Template хоосон бол зааварчилгаа харуулна
    if not template_db["real"] and not template_db["fake"]:
        print("\n⚠  Template олдсонгүй!")
        print("   Эхлээд дараах командыг ажиллуулна уу:")
        print("   python r2_kfs_recognition.py --capture-templates\n")

    cap = cv2.VideoCapture(config["camera_id"])
    block_counter = 0
    last_result   = None
    last_result_time = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        frame = cv2.resize(frame, (640, 480))

        # ─── 1. Ногоон блок олно ───
        blocks, green_mask = find_green_blocks(frame, config)

        # ─── 2. KFS хайрцаг олно ───
        kfs_gray, kfs_bbox = find_kfs_box(frame, config)

        # ─── 3. Template-тай харьцуулна ───
        current_result = None
        if kfs_gray is not None and (template_db["real"] or template_db["fake"]):
            # 0.3 секунд тутамд л шинэ match хийнэ (performance)
            if time.time() - last_result_time > 0.3:
                current_result = match_symbol(kfs_gray, template_db, config)
                last_result    = current_result
                last_result_time = time.time()
            else:
                current_result = last_result

        # ─── 4. Дэлгэц ───
        display = frame.copy()

        # Ногоон блок тойм
        if config["debug"]:
            for cnt in blocks:
                cv2.drawContours(display, [cnt], -1, (0, 200, 0), 1)

        # KFS хайрцаг
        if kfs_bbox is not None:
            x, y, w, h = kfs_bbox
            color = (0, 255, 255)  # default цөнхөр

            if current_result:
                if current_result["label"] == "REAL":
                    color = (0, 255, 0)    # ногоон
                elif current_result["label"] == "FAKE":
                    color = (0, 0, 255)    # улаан

            cv2.rectangle(display, (x, y), (x + w, y + h), color, 2)

        # Үр дүн текст
        if current_result:
            label = current_result["label"]
            conf  = current_result["confidence"]
            name  = current_result["name"]

            if label == "REAL":
                text_color = (0, 220, 0)
                bg_color   = (0, 60, 0)
                action     = "ЦУГЛУУЛ"
            elif label == "FAKE":
                text_color = (0, 0, 220)
                bg_color   = (60, 0, 0)
                action     = "ОРХИ!"
            else:
                text_color = (150, 150, 150)
                bg_color   = (40, 40, 40)
                action     = "ТОДОРХОЙГҮЙ"

            # Арын хайрцаг
            cv2.rectangle(display, (0, 390), (640, 480), bg_color, -1)
            cv2.putText(display, f"{label} — {action}",
                        (10, 425), cv2.FONT_HERSHEY_SIMPLEX, 1.1, text_color, 2)
            cv2.putText(display, f"Char: {name}  |  Conf: {conf:.0%}  |  Matches: {current_result['matches']}",
                        (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.55, text_color, 1)
        else:
            if not kfs_bbox:
                cv2.putText(display, transliterate("KFS харагдахгүй байна..."),
                            (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (120, 120, 120), 1)
            elif not (template_db["real"] or template_db["fake"]):
                cv2.putText(display, transliterate("Template DB хоосон — эхлээд template цуглуул"),
                            (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 100, 255), 1)

        cv2.putText(display, transliterate("S=Хадгал  R=JSON  Q=Гарах"),
                    (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        cv2.imshow("R2 KFS Recognition", display)
        if config["debug"]:
            cv2.imshow("Green mask", green_mask)

        # ─── Товчлуурууд ───
        key = cv2.waitKey(1) & 0xFF

        if key in [ord('q'), 27]:
            break

        elif key == ord('s') and current_result and kfs_gray is not None:
            block_counter += 1
            logger.record(block_counter, current_result, kfs_gray)

        elif key == ord('r'):
            if logger.log:
                logger.save_json()
                logger.summary()
            else:
                print("Хадгалах үр дүн байхгүй байна.")

    cap.release()
    cv2.destroyAllWindows()

    # Гарахдаа автоматаар хадгална
    if logger.log:
        logger.save_json()
        logger.summary()


# ─── ENTRY POINT ──────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import sys

    print("=" * 50)
    print("ABU Robocon 2026 — R2 KFS Recognition System")
    print("=" * 50)

    if "--capture-templates" in sys.argv or "-c" in sys.argv:
        # Горим 1: Template цуглуулах
        capture_templates_mode(CONFIG)
    else:
        # Горим 2: Танилт (recognition)
        run_recognition(CONFIG)


# ─── ЖИШЭЭ ХЭРЭГЛЭГЧИЙН ЗААВАР ───────────────────────────────────────────────
"""
АЛХАМ 1 — Хавтас бэлдэх
─────────────────────────
  mkdir -p kfs_project/templates/real
  mkdir -p kfs_project/templates/fake
  mkdir -p kfs_project/results/detected

АЛХАМ 2 — Template цуглуулах (ХАМГИЙН ЧУХАЛ!)
───────────────────────────────────────────────
  python r2_kfs_recognition.py --capture-templates

  Дараа нь:
  • Rulebook Section 16.2.1 дээрх Real KFS-г камерт харуулна
  • 'r' товчлуур дарж 15 тэмдэгт бүрийг хадгална
  • Fake KFS-г харуулаад 'f' дарна
  • Нийт: 15 real + 15 fake = 30 template

  💡 Зөвлөгөө:
  • Тус тусын KFS хайрцгийг A4 цаасан дээр хэвлэж авна
  • Гэрлийн хэд хэдэн нөхцөлд (гэрэлтэй, харанхуйтсан) template цуглуулна
  • Нэг тэмдэгтийн 3–5 зураг авбал accuracy сайжирна

АЛХАМ 3 — Тест хийх
────────────────────
  python r2_kfs_recognition.py

  Явцад:
  • KFS-г камерт харуул → REAL/FAKE шийдвэр гарна
  • 'S' дарж үр дүнг бүртгэнэ (block ID автоматаар нэмэгдэнэ)
  • 'R' дарж JSON хадгална

АЛХАМ 4 — Pi5 дээр шилжүүлэх
──────────────────────────────
  • CONFIG["camera_id"] = 0 → ийм хэвээр үлдэнэ
  • Raspberry Pi 5 дээр:
      sudo apt install python3-opencv
      python3 r2_kfs_recognition.py

АЛХАМ 5 — Arduino-тай холбох
──────────────────────────────
  import serial
  ser = serial.Serial('/dev/ttyUSB0', 9600)

  # Recognition горим дотор:
  if current_result["label"] == "REAL":
      ser.write(b'COLLECT\\n')   # Arduino → дотогшоо явна
  elif current_result["label"] == "FAKE":
      ser.write(b'AVOID\\n')     # Arduino → зайлана
"""