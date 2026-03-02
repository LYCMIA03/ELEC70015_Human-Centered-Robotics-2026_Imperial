@echo off
chcp 65001 >nul
echo ============================================
echo 垃圾检测系统 - 安装依赖
echo ============================================
echo.

:: 检查 Python
python --version >nul 2>&1
if errorlevel 1 (
    echo [错误] 未找到 Python，请先安装 Python 3.8+
    pause
    exit /b 1
)

echo [1/3] 检测 Python 版本...
python --version

echo.
echo [2/3] 安装基础依赖...
pip install ultralytics opencv-python numpy

echo.
echo [3/3] 检查 PyTorch...
python -c "import torch; print(f'PyTorch: {torch.__version__}'); print(f'CUDA: {torch.cuda.is_available()}')"

if errorlevel 1 (
    echo.
    echo [提示] PyTorch 未安装，请根据你的显卡选择安装:
    echo.
    echo CUDA 11.8:
    echo   pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118
    echo.
    echo CUDA 12.1:
    echo   pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121
    echo.
    echo 仅 CPU:
    echo   pip install torch torchvision
)

echo.
echo ============================================
echo 安装完成！
echo ============================================
echo.
echo 使用方法:
echo   python predict_waste.py 图片路径    - 仅检测垃圾
echo   python predict_smart.py 图片路径    - 智能检测（含人体识别）
echo   python predict_waste.py 0           - 摄像头实时检测
echo.
pause
