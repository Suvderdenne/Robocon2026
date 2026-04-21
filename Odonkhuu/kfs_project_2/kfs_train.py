"""
KFS CNN загвар сургах скрипт
Хэрэглэх: python kfs_train.py
Шаардлага: pip install torch torchvision pillow
"""

import os
import json
import time
import numpy as np
from pathlib import Path

import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms, models
from PIL import Image

# -- Тохиргоо --------------------------------------------------------------
DATA_DIR      = "kfs_data"
MODEL_PATH    = "kfs_model.pt"
CLASSES_FILE  = "kfs_classes.json"
IMG_SIZE      = 64
BATCH_SIZE    = 32
EPOCHS        = 30
LR            = 1e-3
DEVICE        = "cuda" if torch.cuda.is_available() else "cpu"
# --------------------------------------------------------------------------


# -- Dataset ----------------------------------------------------------------
class KFSDataset(Dataset):
    def __init__(self, root, classes, transform=None):
        self.transform = transform
        self.samples = []
        self.classes = classes
        cls2idx = {c: i for i, c in enumerate(classes)}
        for cls in classes:
            d = Path(root) / cls
            if not d.exists():
                continue
            for img_path in d.glob("*.png"):
                self.samples.append((str(img_path), cls2idx[cls]))

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        path, label = self.samples[idx]
        img = Image.open(path).convert("L")   # grayscale
        if self.transform:
            img = self.transform(img)
        return img, label


# -- Загвар (хөнгөн CNN) ----------------------------------------------------
class KFSNet(nn.Module):
    """
    64×64 grayscale орох → анги тоо гарах
    ~150K параметр — Raspberry Pi дээр ч хурдан ажиллана
    """
    def __init__(self, num_classes):
        super().__init__()
        self.features = nn.Sequential(
            # Block 1
            nn.Conv2d(1, 32, 3, padding=1), nn.BatchNorm2d(32), nn.ReLU(),
            nn.Conv2d(32, 32, 3, padding=1), nn.BatchNorm2d(32), nn.ReLU(),
            nn.MaxPool2d(2),       # 32×32
            nn.Dropout2d(0.1),

            # Block 2
            nn.Conv2d(32, 64, 3, padding=1), nn.BatchNorm2d(64), nn.ReLU(),
            nn.Conv2d(64, 64, 3, padding=1), nn.BatchNorm2d(64), nn.ReLU(),
            nn.MaxPool2d(2),       # 16×16
            nn.Dropout2d(0.1),

            # Block 3
            nn.Conv2d(64, 128, 3, padding=1), nn.BatchNorm2d(128), nn.ReLU(),
            nn.Conv2d(128, 128, 3, padding=1), nn.BatchNorm2d(128), nn.ReLU(),
            nn.MaxPool2d(2),       # 8×8
            nn.Dropout2d(0.2),

            # Block 4
            nn.Conv2d(128, 256, 3, padding=1), nn.BatchNorm2d(256), nn.ReLU(),
            nn.MaxPool2d(2),       # 4×4
        )
        self.classifier = nn.Sequential(
            nn.Flatten(),
            nn.Linear(256 * 4 * 4, 512),
            nn.ReLU(),
            nn.Dropout(0.4),
            nn.Linear(512, num_classes),
        )

    def forward(self, x):
        return self.classifier(self.features(x))


# -- Сургалт ----------------------------------------------------------------
def train():
    if not os.path.exists(CLASSES_FILE):
        print("X kfs_classes.json oldsongui. Ehleed kfs_collect.py ajilluul.")
        return

    with open(CLASSES_FILE) as f:
        classes = json.load(f)

    print(f"Tuhuurumj  : {DEVICE}")
    print(f"Angiin too : {len(classes)}")
    print(f"Ugugdliin hawtas: {DATA_DIR}\n")

    # Augmentation
    train_tf = transforms.Compose([
        transforms.Resize((IMG_SIZE, IMG_SIZE)),
        transforms.RandomRotation(15),
        transforms.RandomAffine(0, shear=10, scale=(0.85, 1.15)),
        transforms.RandomHorizontalFlip(),
        transforms.ColorJitter(brightness=0.3, contrast=0.3),
        transforms.ToTensor(),
        transforms.Normalize([0.5], [0.5]),
    ])
    val_tf = transforms.Compose([
        transforms.Resize((IMG_SIZE, IMG_SIZE)),
        transforms.ToTensor(),
        transforms.Normalize([0.5], [0.5]),
    ])

    full_ds = KFSDataset(DATA_DIR, classes, transform=train_tf)
    if len(full_ds) == 0:
        print("X Ugugdul baihgui. Ehleed kfs_collect.py-р Zurag tsugluulah.")
        return

    # 80/20 хуваалт
    val_size  = max(1, int(0.2 * len(full_ds)))
    train_size = len(full_ds) - val_size
    train_ds, val_ds = torch.utils.data.random_split(full_ds, [train_size, val_size])
    val_ds.dataset.transform = val_tf

    train_loader = DataLoader(train_ds, batch_size=BATCH_SIZE, shuffle=True,
                              num_workers=0, pin_memory=(DEVICE == "cuda"))
    val_loader   = DataLoader(val_ds,   batch_size=BATCH_SIZE, shuffle=False,
                              num_workers=0)

    model = KFSNet(num_classes=len(classes)).to(DEVICE)
    criterion = nn.CrossEntropyLoss(label_smoothing=0.1)
    optimizer = optim.AdamW(model.parameters(), lr=LR, weight_decay=1e-4)
    scheduler = optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=EPOCHS)

    best_acc = 0.0
    print("=" * 55)
    print(f"{'Epoch':>6} {'Train Loss':>11} {'Train Acc':>10} {'Val Acc':>8}")
    print("-" * 55)

    for epoch in range(1, EPOCHS + 1):
        # -- Сургалт --
        model.train()
        total_loss, correct, total = 0, 0, 0
        for imgs, labels in train_loader:
            imgs, labels = imgs.to(DEVICE), labels.to(DEVICE)
            optimizer.zero_grad()
            out = model(imgs)
            loss = criterion(out, labels)
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            optimizer.step()
            total_loss += loss.item() * len(imgs)
            correct    += (out.argmax(1) == labels).sum().item()
            total      += len(imgs)
        train_loss = total_loss / total
        train_acc  = correct / total

        # -- Баталгаажуулалт --
        model.eval()
        correct, total = 0, 0
        with torch.no_grad():
            for imgs, labels in val_loader:
                imgs, labels = imgs.to(DEVICE), labels.to(DEVICE)
                out = model(imgs)
                correct += (out.argmax(1) == labels).sum().item()
                total   += len(imgs)
        val_acc = correct / total if total > 0 else 0

        scheduler.step()

        flag = " ★" if val_acc > best_acc else ""
        print(f"{epoch:>6}/{EPOCHS:<4} {train_loss:>11.4f} {train_acc:>9.1%} {val_acc:>7.1%}{flag}")

        if val_acc > best_acc:
            best_acc = val_acc
            torch.save({
                "model_state": model.state_dict(),
                "classes":     classes,
                "img_size":    IMG_SIZE,
                "val_acc":     val_acc,
                "epoch":       epoch,
            }, MODEL_PATH)

    print("=" * 55)
    print(f"\n✓ Hamgiin sain val accuracy: {best_acc:.1%}")
    print(f"✓ Zurag hadgalagdlaa: {MODEL_PATH}")


if __name__ == "__main__":
    train()
