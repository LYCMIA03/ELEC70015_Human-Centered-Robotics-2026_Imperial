# 垃圾检测 (Trash Detection) — 15 类 + RGB-D

基于 YOLOv8 的垃圾分类检测：15 类垃圾 + 可选 RGB-D 相机，支持「最近垃圾 + 最近人」锁定与 3D 坐标输出。

**请勿将 `.pt` 模型文件提交到仓库。** 权重需按下方说明从 Google Drive 下载后放入本地目录使用。

---

## 目录结构

```
trash_detection/
├── weights3/
│   └── epoch80.pt          # 15 类垃圾模型（需自行下载，见下）
├── weights2/
│   └── best.pt             # 可选备用权重
├── predict_15cls.py        # 15 类推理：摄像头 / 图片 / 视频
├── predict_15cls_rgbd.py   # 15 类 + RGB-D：深度、最近人/垃圾、锁定跟随
├── predict_waste.py        # 旧版垃圾检测
├── predict_smart.py        # 旧版智能检测（垃圾+人+携带）
├── requirements.txt
└── README.md
```

---

## 1. 下载 15 类垃圾模型（必做）

模型文件 **不要** 直接上传到 Git，请从 Google Drive 下载后放到本地：

- **下载链接**: [epoch80.pt — Google Drive](https://drive.google.com/file/d/1a3J4hQHD8PBSln1UED4h_vanSC6Mj0xL/view?usp=sharing)
- **放置路径**（二选一）:
  - `trash_detection/weights3/epoch80.pt`（推荐）
  - 或项目上级目录的 `weights3/epoch80.pt`

**命令行下载示例（需安装 gdown）：**

```bash
cd trash_detection
mkdir -p weights3
# 使用 gdown 按 file id 下载（需先 pip install gdown）
gdown "1a3J4hQHD8PBSln1UED4h_vanSC6Mj0xL" -O weights3/epoch80.pt
```

也可在浏览器中打开上述链接，下载后手动将文件移动到 `trash_detection/weights3/epoch80.pt`。

---

## 2. 安装依赖

```bash
cd trash_detection
pip install -r requirements.txt
```

**仅用摄像头/图片/视频（`predict_15cls.py`）** 需要：

- `ultralytics`、`opencv-python`、`numpy`、PyTorch

**使用 RGB-D（`predict_15cls_rgbd.py`）** 还需：

- `pyk4a`（Orbbec Femto Bolt / K4A Wrapper）
- 设置环境变量 `K4A_DLL_DIR` 指向与 k4aviewer 相同的 Orbbec `bin` 目录，例如：
  - PowerShell: `$env:K4A_DLL_DIR = "F:\path\to\OrbbecSDK_K4A_Wrapper_xxx\bin"`

人检测使用 YOLOv8 预训练（默认 `yolov8m.pt`），首次运行时会由 ultralytics 自动下载，无需单独下载。

---

## 3. 使用方法

### 3.1 普通推理：摄像头 / 图片 / 视频（`predict_15cls.py`）

- 15 类检测；Metal 类别有 bias；默认不画 Human；多类别框可重叠。

```bash
# 摄像头
python predict_15cls.py
python predict_15cls.py 0

# 图片
python predict_15cls.py path/to/image.jpg

# 视频
python predict_15cls.py path/to/video.mp4

# 指定权重（若未放在 weights3/epoch80.pt）
python predict_15cls.py --weights path/to/epoch80.pt path/to/image.jpg
```

### 3.2 RGB-D 推理（`predict_15cls_rgbd.py`）

需先下载 15 类模型到 `weights3/epoch80.pt`，并安装 `pyk4a`、配置 `K4A_DLL_DIR`。

- 彩色流默认 1440p；可只显示「最近垃圾 + 最近人」两个框；支持按 **L** 锁定、**U** 解锁，并输出二者 XYZ（米）。

```bash
# 仅 RGB-D 检测（无锁定）
python predict_15cls_rgbd.py

# 最近垃圾 + 最近人，并只画这两个框；按 L 锁定、U 解锁
python predict_15cls_rgbd.py --nearest-person

# 常用可选参数
python predict_15cls_rgbd.py --nearest-person --no-depth-window   # 不显示深度图窗口
python predict_15cls_rgbd.py --nearest-person --max-depth 3       # 只考虑 3 m 内
python predict_15cls_rgbd.py --nearest-person --color-res 1080p  # 彩色 1080p（设备不支持 1440p 时）
python predict_15cls_rgbd.py --nearest-person --person-weights yolov8m.pt  # 人检测模型（默认已是 yolov8m）
```

**按键说明（`--nearest-person` 时）：**

- **L**：锁定当前「最近垃圾」与「离该垃圾最近的人」，之后每帧只画并跟随这两个框，输出其 XYZ。
- **U**：解锁，恢复为每帧重新选最近垃圾与最近人。

---

## 4. 15 类类别

| 0 Metal | 1 Cardboard | 2 Glass | 3 Paper | 4 Plastic | 5 Tetra |
| 6 Apple | 7 Apple-core | 8 Apple-peel | 9 Bread | 10 Orange | 11 Orange-peel |
| 12 Pear | 13 Vegetable | 14 Human |

- `predict_15cls.py`：可画 Human（默认不画，加 `--show-human` 显示）。
- `predict_15cls_rgbd.py` 在 `--nearest-person` 时：15 类模型**只检 0–13（垃圾）**，人由 YOLOv8 预训练（yolov8m.pt）单独检测。

---

## 5. 常见问题

**Q: 提示找不到权重？**  
A: 确保已从 [Google Drive](https://drive.google.com/file/d/1a3J4hQHD8PBSln1UED4h_vanSC6Mj0xL/view?usp=sharing) 下载 `epoch80.pt` 并放到 `trash_detection/weights3/epoch80.pt`，或使用 `--weights path/to/epoch80.pt`。

**Q: 不要提交 `.pt` 到 Git？**  
A: 本仓库已通过 `.gitignore` 忽略 `*.pt`，请勿将模型文件加入版本控制。

**Q: RGB-D 启动失败？**  
A: 检查 `K4A_DLL_DIR` 是否指向 Orbbec 的 `bin` 目录，且与 k4aviewer 使用一致；若设备不支持 1440p，可加 `--color-res 1080p`。

**Q: 检测速度慢？**  
A: 安装 CUDA 版 PyTorch；人检测可改用更小模型，例如 `--person-weights yolov8n.pt`。

---

## 6. 许可证

本项目仅供学习研究使用。
