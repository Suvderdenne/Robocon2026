"""
KFS (Kung-Fu Scroll) тэмдэгт цуглуулах скрипт
Хэрэглэх: python kfs_collect.py
"""

import cv2
import os
import time
import json
import numpy as np
from pathlib import Path

# -- Тохиргоо --------------------------------------------------------------
IMG_SIZE    = 64          # CNN-д өгөх зургийн хэмжээ
DATA_DIR    = "kfs_data"  # Өгөгдөл hadgalah хавтас
CLASSES_FILE = "kfs_classes.json"

# 15 жинхэнэ + 15 хуурамч + 1 хоосон (KFS биш) = 31 angi
# Өөрийн тэмдэгтүүдийн нэрийг энд нэмнэ
DEFAULT_CLASSES = [
    # R2 KFS (Oracle Bone Characters) - жинхэнэ
    "char_01", "char_02", "char_03", "char_04", "char_05",
    "char_06", "char_07", "char_08", "char_09", "char_10",
    "char_11", "char_12", "char_13", "char_14", "char_15",
    # Хуурамч KFS
    "fake_01", "fake_02", "fake_03", "fake_04", "fake_05",
    "fake_06", "fake_07", "fake_08", "fake_09", "fake_10",
    "fake_11", "fake_12", "fake_13", "fake_14", "fake_15",
    # KFS биш (арын дэвсгэр гэх мэт)
    "background"
]
# --------------------------------------------------------------------------


def ensure_dirs(classes):
    for cls in classes:
        Path(f"{DATA_DIR}/{cls}").mkdir(parents=True, exist_ok=True)
    print(f"✓ Havtas Uusgesen: {DATA_DIR}/")


def load_classes():
    if os.path.exists(CLASSES_FILE):
        with open(CLASSES_FILE) as f:
            return json.load(f)
    with open(CLASSES_FILE, "w") as f:
        json.dump(DEFAULT_CLASSES, f, ensure_ascii=False, indent=2)
    return DEFAULT_CLASSES


def crop_kfs_box(frame):
    """
    Дэлгэцнээс KFS хайрцгийг илрүүлэн тайрна.
    Хэрэв илрүүлж чадахгүй бол бүтэн фрэймийн төв хэсгийг буцаана.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Цагаан/цайвар өнгийн KFS блок илрүүлэх
    mask = cv2.inRange(hsv, (0, 0, 180), (180, 50, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                            np.ones((7, 7), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                               cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for c in cnts:
        area = cv2.contourArea(c)
        if area < 2000:
            continue
        x, y, w, h = cv2.boundingRect(c)
        ratio = w / max(h, 1)
        score = area * (1 - abs(ratio - 1))   # квадрат хэлбэртэй байх тусам өндөр
        if best is None or score > best[0]:
            best = (score, x, y, w, h)

    if best:
        _, x, y, w, h = best
        pad = 8
        x, y = max(0, x - pad), max(0, y - pad)
        w, h = min(frame.shape[1] - x, w + 2*pad), min(frame.shape[0] - y, h + 2*pad)
        return frame[y:y+h, x:x+w], (x, y, w, h)

    # Fallback: голын тэгш дөрвөлжин
    h, w = frame.shape[:2]
    side = min(h, w) // 2
    cy, cx = h // 2, w // 2
    crop = frame[cy - side//2: cy + side//2, cx - side//2: cx + side//2]
    return crop, (cx - side//2, cy - side//2, side, side)


def preprocess(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.resize(gray, (IMG_SIZE, IMG_SIZE))
    gray = cv2.equalizeHist(gray)
    return gray


def collect():
    classes = load_classes()
    ensure_dirs(classes)

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    current_idx = 0
    count_map = {cls: len(list(Path(f"{DATA_DIR}/{cls}").glob("*.png")))
                 for cls in classes}

    print("\n--- KFS Ugugdul tsugluulah ---")
    print("  ← →  : angi solikh")
    print("  SPACE : zurag hadgalah (auto burst)")
    print("  Q     : garah\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        crop, bbox = crop_kfs_box(frame)
        x, y, w, h = bbox

        # Визуал
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cls_name   = classes[current_idx]
        count      = count_map[cls_name]
        cv2.putText(frame, f"Angi: {cls_name}  ({count} zurag)",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
        cv2.putText(frame, "SPACE=hadgalah  </>= angi  Q=garah",
                    (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        preview = cv2.resize(crop, (150, 150))
        frame[10:160, 470:620] = preview if preview.shape == (150, 150, 3) else frame[10:160, 470:620]

        cv2.imshow("KFS tsugluulagch", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord(',') or key == 81:   # ←
            current_idx = (current_idx - 1) % len(classes)
        elif key == ord('.') or key == 83:   # →
            current_idx = (current_idx + 1) % len(classes)
        elif key == ord(' '):
            # Burst: 5 зураг дараалан хадгал (бага зэрэг өнцөг зөрүүтэй)
            cls_dir = f"{DATA_DIR}/{classes[current_idx]}"
            for i in range(5):
                ts = int(time.time() * 1000) + i
                proc = preprocess(crop)
                # Өнцөг эргүүлэлт (data augmentation)
                angle = np.random.uniform(-10, 10)
                M = cv2.getRotationMatrix2D((IMG_SIZE//2, IMG_SIZE//2), angle, 1)
                proc = cv2.warpAffine(proc, M, (IMG_SIZE, IMG_SIZE))
                cv2.imwrite(f"{cls_dir}/{ts}.png", proc)
            count_map[classes[current_idx]] += 5
            print(f"  ✓ {classes[current_idx]}: {count_map[classes[current_idx]]} zurag")

    cap.release()
    cv2.destroyAllWindows()
    print("\n--- Niit tsugluulsan ---")
    for cls in classes:
        print(f"  {cls}: {count_map[cls]}")


if __name__ == "__main__":
    collect()
