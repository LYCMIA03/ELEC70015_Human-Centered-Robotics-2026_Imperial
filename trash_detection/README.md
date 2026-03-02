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

### 2.1 Jetson 上使用 GPU（PyTorch CUDA）

若在 **Jetson（JetPack 6.x）** 上运行，`pip install torch` 装的是 CPU 版，需改用 NVIDIA 官方带 CUDA 的 wheel 才能让 YOLO 用 GPU：

```bash
# 先装系统依赖（若未装）
sudo apt-get -y update
sudo apt-get install -y python3-pip libopenblas-dev

# 卸掉当前 CPU 版 PyTorch
pip3 uninstall -y torch

# 安装 NVIDIA 提供的 PyTorch CUDA wheel（JetPack 6.1/6.2，Python 3.10）
pip3 install --no-cache-dir https://developer.download.nvidia.com/compute/redist/jp/v61/pytorch/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl
```

若导入 torch 报错 **`libcusparseLt.so.0: cannot open shared object file`**，需先安装 cuSPARSELt（NVIDIA 单独提供）。

**方式 A（推荐，无需 sudo）**：下载后解压到用户目录，并用 `LD_LIBRARY_PATH` 指向其 `lib`：

```bash
cd /tmp
wget https://developer.download.nvidia.com/compute/cusparselt/redist/libcusparse_lt/linux-aarch64/libcusparse_lt-linux-aarch64-0.8.1.1_cuda12-archive.tar.xz
tar xf libcusparse_lt-linux-aarch64-0.8.1.1_cuda12-archive.tar.xz
mkdir -p ~/.local/cusparselt
cp -P libcusparse_lt-linux-aarch64-0.8.1.1_cuda12-archive/lib/* ~/.local/cusparselt/
echo 'export LD_LIBRARY_PATH="$HOME/.local/cusparselt:$LD_LIBRARY_PATH"' >> ~/.bashrc
source ~/.bashrc
```

**方式 B（系统路径，需 sudo）**：

```bash
cd /tmp
# 若未下载：wget ... 与 tar xf ...（同上）
sudo cp -P libcusparse_lt-linux-aarch64-0.8.1.1_cuda12-archive/lib/* /usr/local/cuda-12.6/targets/aarch64-linux/lib/
sudo ldconfig
```

安装后验证：`python3 -c "import torch; print('CUDA:', torch.cuda.is_available())"` 应输出 `CUDA: True`。

**说明（Jetson + 上述 wheel）**：  
- **torchvision**：PyPI 的 torchvision 0.20/0.25 与 Jetson 专用 torch 2.5.0a0 在 C++ 扩展上不兼容（会报 `operator torchvision::nms does not exist`）。请安装 **torchvision 0.15.2** 并与 Jetson wheel 搭配使用：`pip3 install 'torchvision==0.15.2' --no-deps`（pip 会提示版本不兼容，可忽略；ultralytics YOLO 可正常用）。  
- **NumPy**：若出现 NumPy 2.x 与 torch 不兼容的警告或报错，可执行 `pip3 install 'numpy<2'`。  
- **NVML / CUDACachingAllocator 报错（间歇性）**：Jetson 上该错误有时出现、有时不出现。脚本已做：(1) 开相机前 YOLO CUDA 预热；(2) 每帧若遇此错误会**自动重试最多 3 次**（仍用 CUDA），多数情况下重试即可通过。若仍频繁崩溃可设 `PYTORCH_CUDA_ALLOC_CONF=backend:native` 或减小 `--imgsz`（如 640）。  
更多版本见 [NVIDIA 文档](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform-release-notes/pytorch-jetson-rel.html)。

### 2.2 让 torchvision 与 Jetson PyTorch 真正兼容（可选）

当前脚本已对 `torchvision.ops.nms` 做了回退（失败时用 ultralytics 自带的 NMS），因此用 **torchvision 0.15.2** 即可正常运行。若希望**彻底兼容**、去掉版本警告和 C++ 扩展加载失败，可在 Jetson 上**从源码编译**与已装 PyTorch 2.5.0a0 匹配的 torchvision（需约 20–40 分钟）：

```bash
# 依赖（若未装）
sudo apt-get install -y git cmake ninja-build libjpeg-dev zlib1g-dev

# 卸载当前 torchvision，避免与即将编译的冲突
pip3 uninstall -y torchvision

# 克隆与 PyTorch 2.5 对应的 torchvision 0.20 并编译（Jetson 建议单线程，避免内存不足）
cd /tmp
git clone --depth 1 --branch v0.20.0 https://github.com/pytorch/vision.git
cd vision
export FORCE_CUDA=1
MAX_JOBS=1 pip3 install . --no-build-isolation
cd ..
rm -rf vision
```

编译完成后执行 `python3 -c "import torchvision; print(torchvision.__version__); torchvision.ops.nms(torch.rand(2,4), torch.rand(2), 0.5)"` 若无报错即表示兼容。之后可去掉脚本里对 `torchvision.ops.nms` 的回退补丁（若你希望完全依赖系统 torchvision）。

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

### 3.3 非 ROS 推理 + Docker ROS 桥接（推荐 Jetson + Noetic 容器）

`predict_15cls_rgbd.py` 可直接通过 UDP 发送 XYZ（不依赖 ROS）：

```bash
python trash_detection/predict_15cls_rgbd.py \
  --nearest-person --print-xyz --headless \
  --udp-enable --udp-host 127.0.0.1 --udp-port 16031 \
  --udp-frame-id camera_link --udp-kind waste
```

也可用封装脚本：

```bash
./scripts/start_trash_detection_rgbd.sh
```

默认发送 JSON 字段：`stamp, frame_id, x, y, z, source, kind`。

Docker(Noetic) 侧使用 `target_follower/udp_target_bridge.py` 接收 UDP 并发布：

- 输入：UDP `127.0.0.1:16031`（默认）
- 输出：`/trash_detection/target_point` (`geometry_msgs/PointStamped`)

再通过 `point_to_target_pose.py` 转成 `/target_pose`，交给 `target_follower.py`。

> 端口 `16031` 与 ROS master `11311` 完全独立，不会冲突。

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
