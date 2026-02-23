"""
智能垃圾检测 + 人体识别
功能：
- 检测垃圾
- 检测人
- 当人和垃圾框重叠时，判定为"人携带垃圾"
- 过滤误检（人被误识别为垃圾的情况）

用法：
    python predict_smart.py 图片路径      # 检测图片
    python predict_smart.py 0            # 使用摄像头实时检测
"""
from ultralytics import YOLO
import cv2
import sys
from pathlib import Path

# 加载模型
WASTE_MODEL_PATH = 'weights/waste_best.pt'
PERSON_MODEL_PATH = 'weights/yolov8s.pt'

waste_model = YOLO(WASTE_MODEL_PATH)   # 垃圾分类模型
person_model = YOLO(PERSON_MODEL_PATH)  # COCO 预训练模型（检测人）

# 垃圾类别
WASTE_CLASSES = ['Can', 'Cardboard', 'Carton', 'Glass', 'Newspaper', 'Paper', 'Plastic', 'Tetra', 
                 'Apple', 'Apple-core', 'Apple-peel', 'Bone', 'Bone-fish', 'Bread', 'Egg-shell', 
                 'Egg-yolk', 'Meat', 'Noodle', 'Orange', 'Orange-peel', 'Pear', 'Rice', 'Vegetable']

# 禁用的类别（表现不佳）
DISABLED_CLASSES = ['Newspaper', 'Rice']
ENABLED_CLASS_IDS = [i for i, name in enumerate(WASTE_CLASSES) if name not in DISABLED_CLASSES]

# 配置
CONF_THRESHOLD = 0.25          # 垃圾检测置信度阈值
PERSON_CONF = 0.5              # 人体检测置信度
OVERLAP_THRESHOLD = 0.1        # 判定为"携带"的重叠阈值
MISDETECT_IOU_THRESHOLD = 0.7  # 误检过滤阈值
SIZE_RATIO_THRESHOLD = 0.5     # 垃圾框/人框面积比阈值

# 颜色配置 (BGR)
COLOR_WASTE = (0, 255, 0)      # 绿色 - 普通垃圾
COLOR_PERSON = (255, 0, 0)     # 蓝色 - 人
COLOR_CARRYING = (0, 165, 255) # 橙色 - 人携带垃圾


def calculate_iou(box1, box2):
    """计算两个框的 IoU"""
    x1 = max(box1[0], box2[0])
    y1 = max(box1[1], box2[1])
    x2 = min(box1[2], box2[2])
    y2 = min(box1[3], box2[3])
    
    intersection = max(0, x2 - x1) * max(0, y2 - y1)
    area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
    area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
    union = area1 + area2 - intersection
    
    return intersection / union if union > 0 else 0


def calculate_overlap_ratio(small_box, large_box):
    """计算小框被大框覆盖的比例"""
    x1 = max(small_box[0], large_box[0])
    y1 = max(small_box[1], large_box[1])
    x2 = min(small_box[2], large_box[2])
    y2 = min(small_box[3], large_box[3])
    
    intersection = max(0, x2 - x1) * max(0, y2 - y1)
    small_area = (small_box[2] - small_box[0]) * (small_box[3] - small_box[1])
    
    return intersection / small_area if small_area > 0 else 0


def calculate_box_area(box):
    """计算框的面积"""
    return (box[2] - box[0]) * (box[3] - box[1])


def is_likely_misdetect(waste_box, person_box, iou):
    """判断是否为误检（把人识别成垃圾）"""
    waste_area = calculate_box_area(waste_box)
    person_area = calculate_box_area(person_box)
    
    if person_area == 0:
        return False
    
    size_ratio = waste_area / person_area
    return iou > MISDETECT_IOU_THRESHOLD and size_ratio > SIZE_RATIO_THRESHOLD


def analyze_detections(waste_boxes, waste_classes, waste_confs, person_boxes):
    """分析检测结果"""
    results = {
        'normal_waste': [],
        'carried_waste': [],
        'misdetect': [],
        'persons': []
    }
    
    for i, pbox in enumerate(person_boxes):
        results['persons'].append({
            'box': pbox,
            'carrying': []
        })
    
    for i, (wbox, cls_id, conf) in enumerate(zip(waste_boxes, waste_classes, waste_confs)):
        max_overlap = 0
        max_iou = 0
        associated_person = -1
        associated_pbox = None
        
        for j, pbox in enumerate(person_boxes):
            overlap = calculate_overlap_ratio(wbox, pbox)
            iou = calculate_iou(wbox, pbox)
            if overlap > max_overlap:
                max_overlap = overlap
                max_iou = iou
                associated_person = j
                associated_pbox = pbox
        
        if associated_pbox is not None and is_likely_misdetect(wbox, associated_pbox, max_iou):
            results['misdetect'].append((wbox, cls_id, conf))
        elif max_overlap > OVERLAP_THRESHOLD:
            results['carried_waste'].append((wbox, cls_id, conf, associated_person))
            results['persons'][associated_person]['carrying'].append({
                'box': wbox,
                'class': WASTE_CLASSES[int(cls_id)],
                'conf': conf
            })
        else:
            results['normal_waste'].append((wbox, cls_id, conf))
    
    return results


def draw_results(frame, results):
    """绘制检测结果"""
    annotated = frame.copy()
    
    # 绘制人
    for person in results['persons']:
        box = person['box']
        x1, y1, x2, y2 = map(int, box)
        
        if person['carrying']:
            cv2.rectangle(annotated, (x1, y1), (x2, y2), COLOR_CARRYING, 3)
            waste_names = [w['class'] for w in person['carrying']]
            label = f"Person carrying: {', '.join(waste_names)}"
            (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(annotated, (x1, y1-25), (x1+w, y1), COLOR_CARRYING, -1)
            cv2.putText(annotated, label, (x1, y1-7), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        else:
            cv2.rectangle(annotated, (x1, y1), (x2, y2), COLOR_PERSON, 2)
            cv2.putText(annotated, 'Person', (x1, y1-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_PERSON, 2)
    
    # 绘制普通垃圾
    for wbox, cls_id, conf in results['normal_waste']:
        x1, y1, x2, y2 = map(int, wbox)
        label = f'{WASTE_CLASSES[int(cls_id)]} {conf:.2f}'
        cv2.rectangle(annotated, (x1, y1), (x2, y2), COLOR_WASTE, 2)
        cv2.putText(annotated, label, (x1, y1-10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_WASTE, 2)
    
    # 绘制携带的垃圾
    for wbox, cls_id, conf, _ in results['carried_waste']:
        x1, y1, x2, y2 = map(int, wbox)
        label = f'{WASTE_CLASSES[int(cls_id)]} {conf:.2f}'
        cv2.rectangle(annotated, (x1, y1), (x2, y2), COLOR_CARRYING, 2)
        cv2.putText(annotated, label, (x1, y1-10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_CARRYING, 2)
    
    # 统计信息
    info_y = 30
    cv2.putText(annotated, f"Persons: {len(results['persons'])}", (10, info_y), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_PERSON, 2)
    cv2.putText(annotated, f"Waste: {len(results['normal_waste'])}", (10, info_y+25), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_WASTE, 2)
    cv2.putText(annotated, f"Carrying: {len(results['carried_waste'])}", (10, info_y+50), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_CARRYING, 2)
    
    return annotated


def predict_smart(source, show=True, save=True):
    """智能检测"""
    
    is_camera = source == 0 or source == '0'
    
    if is_camera:
        cap = cv2.VideoCapture(0)
        print("摄像头已启动，按 'q' 退出")
        
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            # 检测人
            person_results = person_model.predict(frame, classes=[0], conf=PERSON_CONF, verbose=False)
            person_boxes = []
            if len(person_results[0].boxes) > 0:
                person_boxes = person_results[0].boxes.xyxy.cpu().numpy()
            
            # 检测垃圾
            waste_results = waste_model.predict(frame, classes=ENABLED_CLASS_IDS, conf=CONF_THRESHOLD, verbose=False)
            waste_boxes = []
            waste_classes = []
            waste_confs = []
            if len(waste_results[0].boxes) > 0:
                waste_boxes = waste_results[0].boxes.xyxy.cpu().numpy()
                waste_classes = waste_results[0].boxes.cls.cpu().numpy()
                waste_confs = waste_results[0].boxes.conf.cpu().numpy()
            
            # 分析结果
            results = analyze_detections(waste_boxes, waste_classes, waste_confs, person_boxes)
            annotated_frame = draw_results(frame, results)
            
            cv2.imshow('Smart Waste Detection', annotated_frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        cap.release()
        cv2.destroyAllWindows()
    
    else:
        frame = cv2.imread(source)
        if frame is None:
            print(f"错误: 无法读取图片 {source}")
            return
        
        # 检测
        person_results = person_model.predict(frame, classes=[0], conf=PERSON_CONF, verbose=False)
        person_boxes = []
        if len(person_results[0].boxes) > 0:
            person_boxes = person_results[0].boxes.xyxy.cpu().numpy()
        
        waste_results = waste_model.predict(frame, classes=ENABLED_CLASS_IDS, conf=CONF_THRESHOLD, verbose=False)
        waste_boxes = []
        waste_classes = []
        waste_confs = []
        if len(waste_results[0].boxes) > 0:
            waste_boxes = waste_results[0].boxes.xyxy.cpu().numpy()
            waste_classes = waste_results[0].boxes.cls.cpu().numpy()
            waste_confs = waste_results[0].boxes.conf.cpu().numpy()
        
        results = analyze_detections(waste_boxes, waste_classes, waste_confs, person_boxes)
        
        print(f"\n检测结果:")
        print(f"  人: {len(results['persons'])}")
        print(f"  普通垃圾: {len(results['normal_waste'])}")
        print(f"  人携带的垃圾: {len(results['carried_waste'])}")
        
        for i, person in enumerate(results['persons']):
            if person['carrying']:
                waste_list = [f"{w['class']}({w['conf']:.2f})" for w in person['carrying']]
                print(f"  -> 人 {i+1} 携带: {', '.join(waste_list)}")
        
        annotated_frame = draw_results(frame, results)
        
        if save:
            output_dir = Path('output')
            output_dir.mkdir(parents=True, exist_ok=True)
            output_path = output_dir / Path(source).name
            cv2.imwrite(str(output_path), annotated_frame)
            print(f"\n结果已保存: {output_path}")
        
        if show:
            cv2.imshow('Smart Waste Detection', annotated_frame)
            cv2.waitKey(0)
            cv2.destroyAllWindows()


if __name__ == '__main__':
    print("="*50)
    print("智能垃圾检测系统")
    print("="*50)
    print(f"功能:")
    print(f"  - 检测垃圾和人")
    print(f"  - 识别人携带垃圾的场景")
    print(f"  - 过滤误检")
    print(f"\n颜色说明:")
    print(f"  绿色 - 普通垃圾")
    print(f"  蓝色 - 人")
    print(f"  橙色 - 人携带垃圾")
    print("="*50)
    
    if len(sys.argv) > 1:
        source = sys.argv[1]
        if source.isdigit():
            source = int(source)
    else:
        source = 0
    
    print(f"\n输入源: {source}")
    predict_smart(source)
