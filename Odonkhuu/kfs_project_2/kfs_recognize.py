"""
KFS CNN бодит цаг Recognition
Хэрэглэх: python kfs_recognize.py
           python kfs_recognize.py --image test.jpg   (нэг зураг шалгах)
           python kfs_recognize.py --eval             (өгөгдлийн accuracy шалгах)
"""

import cv2
import json
import time
import argparse
import numpy as np
from pathlib import Path
from collections import deque

import torch
import torch.nn as nn
from torchvision import transforms
from PIL import Image

# -- Тохиргоо --------------------------------------------------------------
MODEL_PATH      = "kfs_model.pt"
IMG_SIZE        = 64
CONF_THRESHOLD  = 0.75    # Энэнээс өндөр итгэлцэл байж мэдэгдэнэ
SMOOTH_FRAMES   = 8       # Smoothing-д хэдэн фрэйм ашиглах
CAM_INDEX       = 0
# --------------------------------------------------------------------------


# -- Загвар (train.py-тай ижил байх ёстой) ---------------------------------
class KFSNet(nn.Module):
    def __init__(self, num_classes):
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(1, 32, 3, padding=1), nn.BatchNorm2d(32), nn.ReLU(),
            nn.Conv2d(32, 32, 3, padding=1), nn.BatchNorm2d(32), nn.ReLU(),
            nn.MaxPool2d(2), nn.Dropout2d(0.1),

            nn.Conv2d(32, 64, 3, padding=1), nn.BatchNorm2d(64), nn.ReLU(),
            nn.Conv2d(64, 64, 3, padding=1), nn.BatchNorm2d(64), nn.ReLU(),
            nn.MaxPool2d(2), nn.Dropout2d(0.1),

            nn.Conv2d(64, 128, 3, padding=1), nn.BatchNorm2d(128), nn.ReLU(),
            nn.Conv2d(128, 128, 3, padding=1), nn.BatchNorm2d(128), nn.ReLU(),
            nn.MaxPool2d(2), nn.Dropout2d(0.2),

            nn.Conv2d(128, 256, 3, padding=1), nn.BatchNorm2d(256), nn.ReLU(),
            nn.MaxPool2d(2),
        )
        self.classifier = nn.Sequential(
            nn.Flatten(),
            nn.Linear(256 * 4 * 4, 512), nn.ReLU(), nn.Dropout(0.4),
            nn.Linear(512, num_classes),
        )

    def forward(self, x):
        return self.classifier(self.features(x))
# --------------------------------------------------------------------------


class KFSRecognizer:
    def __init__(self, model_path=MODEL_PATH):
        if not Path(model_path).exists():
            raise FileNotFoundError(
                f"Zagwar oldsongui: {model_path}\n"
                "Ehleed surga: python kfs_train.py"
            )
        ckpt          = torch.load(model_path, map_location="cpu")
        self.classes  = ckpt["classes"]
        self.img_size = ckpt.get("img_size", IMG_SIZE)
        self.device   = "cuda" if torch.cuda.is_available() else "cpu"

        self.model = KFSNet(num_classes=len(self.classes)).to(self.device)
        self.model.load_state_dict(ckpt["model_state"])
        self.model.eval()

        self.transform = transforms.Compose([
            transforms.Resize((self.img_size, self.img_size)),
            transforms.ToTensor(),
            transforms.Normalize([0.5], [0.5]),
        ])

        self.history  = deque(maxlen=SMOOTH_FRAMES)
        val_acc = ckpt.get("val_acc", 0)
        print(f"✓ Zagwar achaalagdlaa  ({len(self.classes)} angi, val_acc={val_acc:.1%})")

    # -- KFS блок илрүүлэх ------------------------------------------------
    @staticmethod
    def detect_box(frame):
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 0, 170), (180, 60, 255))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  np.ones((5, 5), np.uint8))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        for c in cnts:
            area = cv2.contourArea(c)
            if area < 1500:
                continue
            peri  = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.03 * peri, True)
            x, y, w, h = cv2.boundingRect(c)
            ratio = w / max(h, 1)
            # Квадрат хэлбэрт ойр байх тусам score өндөр
            score = area * min(ratio, 1/ratio) * (1 + (len(approx) == 4) * 0.5)
            if best is None or score > best[0]:
                best = (score, x, y, w, h)

        if best:
            _, x, y, w, h = best
            pad = 10
            x1 = max(0, x - pad);  y1 = max(0, y - pad)
            x2 = min(frame.shape[1], x + w + pad)
            y2 = min(frame.shape[0], y + h + pad)
            return frame[y1:y2, x1:x2], (x1, y1, x2 - x1, y2 - y1)

        # Fallback
        h, w = frame.shape[:2]
        s = min(h, w) // 2
        cx, cy = w // 2, h // 2
        return frame[cy-s//2:cy+s//2, cx-s//2:cx+s//2], (cx-s//2, cy-s//2, s, s)

    # -- Нэг зураг classification ------------------------------------------
    @torch.no_grad()
    def classify(self, crop_bgr):
        gray = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        img  = Image.fromarray(gray)
        inp  = self.transform(img).unsqueeze(0).to(self.device)

        logits = self.model(inp)
        probs  = torch.softmax(logits, dim=1)[0]
        idx    = probs.argmax().item()
        return self.classes[idx], probs[idx].item(), probs.cpu().numpy()

    # -- Smoothed prediction -----------------------------------------------
    def predict_smooth(self, frame):
        crop, bbox = self.detect_box(frame)
        if crop.size == 0:
            return "ilersengui", 0.0, bbox, None

        cls, conf, probs = self.classify(crop)
        self.history.append(probs)

        # Хэд хэдэн фрэймийн дундаж итгэлцэл
        avg_probs = np.mean(list(self.history), axis=0)
        idx  = int(np.argmax(avg_probs))
        conf = float(avg_probs[idx])
        return self.classes[idx], conf, bbox, crop

    # -- Бодит цаг ----------------------------------------------------------
    def run_live(self):
        cap = cv2.VideoCapture(CAM_INDEX)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        print("Camera neegdlee, Q darj garna.")

        fps_time = time.time()
        fps = 0

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            cls, conf, (x, y, w, h), crop = self.predict_smooth(frame)

            # -- Дэлгэц --
            is_fake = cls.startswith("fake")
            is_bg   = cls == "background"
            color   = (0, 0, 220) if is_fake else (0, 200, 0) if not is_bg else (150, 150, 150)

            cv2.rectangle(frame, (x, y), (x+w, y+h), color, 2)

            label = cls if conf >= CONF_THRESHOLD else f"? ({conf:.0%})"
            if conf >= CONF_THRESHOLD:
                label = f"{'🔴 FAKE' if is_fake else '✅ REAL'}: {cls}  {conf:.0%}"

            # Хар дэвсгэртэй текст
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
            cv2.rectangle(frame, (x, y - th - 12), (x + tw + 8, y), color, -1)
            cv2.putText(frame, label, (x + 4, y - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

            # FPS
            fps = 0.9 * fps + 0.1 * (1 / max(time.time() - fps_time, 1e-5))
            fps_time = time.time()
            cv2.putText(frame, f"FPS: {fps:.1f}", (10, 460),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)

            # Preview
            if crop is not None and crop.size > 0:
                prev = cv2.resize(crop, (100, 100))
                frame[10:110, 530:630] = prev

            cv2.imshow("KFS Recognition (CNN)", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

    # -- Нэг зураг шалгах --------------------------------------------------
    def predict_image(self, path):
        frame = cv2.imread(path)
        if frame is None:
            print(f"X Zurag Neegdsengui: {path}")
            return
        cls, conf, bbox, _ = self.predict_smooth(frame)
        print(f"\nZurag   : {path}")
        print(f"Angi    : {cls}")
        print(f"Itgeltsel: {conf:.1%}")
        print(f"Dun     : {'FAKE ⚠' if cls.startswith('fake') else 'REAL ✓'}")
        cv2.imshow("Result", frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    # -- Eval --------------------------------------------------------------
    def evaluate(self, data_dir="kfs_data"):
        correct = total = 0
        errors  = []
        for cls in self.classes:
            d = Path(data_dir) / cls
            if not d.exists():
                continue
            for img_path in d.glob("*.png"):
                img = cv2.imread(str(img_path))
                if img is None:
                    continue
                pred, conf, _, _ = self.predict_smooth(img)
                self.history.clear()   # фрэйм smoothing цэвэрлэх
                total += 1
                if pred == cls:
                    correct += 1
                else:
                    errors.append((str(img_path), cls, pred, conf))

        acc = correct / total if total > 0 else 0
        print(f"\n-- Evaluation --")
        print(f"Niit   : {total}")
        print(f"Zuw    : {correct}")
        print(f"Accuracy: {acc:.1%}")
        if errors:
            print(f"\nAldaa ({len(errors)}):")
            for path, true, pred, conf in errors[:10]:
                print(f"  {Path(path).name:20s}  unen={true:12s} taasan={pred:12s} conf={conf:.0%}")


# -- Гол -----------------------------------------------------------------
def main():
    p = argparse.ArgumentParser(description="KFS CNN Recognition")
    p.add_argument("--image", default=None, help="Neg zurag shalgah")
    p.add_argument("--eval",  action="store_true", help="Eval gorim")
    args = p.parse_args()

    rec = KFSRecognizer()

    if args.image:
        rec.predict_image(args.image)
    elif args.eval:
        rec.evaluate()
    else:
        rec.run_live()


if __name__ == "__main__":
    main()
