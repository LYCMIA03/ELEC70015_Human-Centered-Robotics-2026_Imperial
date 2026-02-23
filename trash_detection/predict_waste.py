"""
垃圾检测脚本 - 仅检测垃圾
用法：
    python predict_waste.py 图片路径      # 检测图片
    python predict_waste.py 0            # 使用摄像头实时检测
"""
from ultralytics import YOLO
import cv2
import sys
from pathlib import Path

# 加载垃圾分类模型
MODEL_PATH = 'weights/waste_best.pt'
model = YOLO(MODEL_PATH)

# 垃圾类别
WASTE_CLASSES = ['Can', 'Cardboard', 'Carton', 'Glass', 'Newspaper', 'Paper', 'Plastic', 'Tetra', 
                 'Apple', 'Apple-core', 'Apple-peel', 'Bone', 'Bone-fish', 'Bread', 'Egg-shell', 
                 'Egg-yolk', 'Meat', 'Noodle', 'Orange', 'Orange-peel', 'Pear', 'Rice', 'Vegetable']

# 配置
CONF_THRESHOLD = 0.25  # 置信度阈值


def predict(source, show=True, save=True):
    """检测垃圾"""
    
    is_camera = source == 0 or source == '0'
    
    if is_camera:
        cap = cv2.VideoCapture(0)
        print("摄像头已启动，按 'q' 退出")
        
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            results = model.predict(frame, conf=CONF_THRESHOLD, verbose=False)
            annotated_frame = results[0].plot()
            
            cv2.imshow('Waste Detection', annotated_frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        cap.release()
        cv2.destroyAllWindows()
    
    else:
        # 处理图片
        results = model.predict(source, conf=CONF_THRESHOLD)
        
        if save:
            output_dir = Path('output')
            output_dir.mkdir(parents=True, exist_ok=True)
            output_path = output_dir / Path(source).name
            
            annotated = results[0].plot()
            cv2.imwrite(str(output_path), annotated)
            print(f"结果已保存: {output_path}")
        
        if show:
            annotated = results[0].plot()
            cv2.imshow('Result', annotated)
            cv2.waitKey(0)
            cv2.destroyAllWindows()


if __name__ == '__main__':
    print("="*50)
    print("垃圾检测系统")
    print("="*50)
    print(f"模型: {MODEL_PATH}")
    print(f"类别数: {len(WASTE_CLASSES)}")
    print(f"置信度阈值: {CONF_THRESHOLD}")
    print("="*50)
    
    if len(sys.argv) > 1:
        source = sys.argv[1]
        if source.isdigit():
            source = int(source)
    else:
        source = 0
    
    print(f"\n输入源: {source}")
    predict(source)
