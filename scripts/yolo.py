from ultralytics import YOLO
import cv2
import numpy as np
import io
import base64
from PIL import Image

# 加载预训练模型
model = YOLO("src/aubo_grasp/weights/best.pt")

# 检测图片
def get_all_objects_info(img):
    objects_info = []
    results = model(img)
    id=1
    for result in results[0].boxes:
        x1 = int(np.int64(result.xyxy[0][0].item()))
        y1 = int(np.int64(result.xyxy[0][1].item()))
        x2 = int(np.int64(result.xyxy[0][2].item()))
        y2 = int(np.int64(result.xyxy[0][3].item()))
        cls = results[0].names[result.cls.item()]
        probability = result.conf.item()
        box=(x1, y1, x2, y2)
        buffer = io.BytesIO()
        Image.open(img).crop(box).save(buffer, format='PNG')

        objects_info.append({
            'id': id,
            'class': cls,
            'color': "",
            'position': (x1, y1, x2, y2),
            'image':base64.b64encode(buffer.getvalue()).decode('utf-8'),
            'confidence': probability
        })

    return objects_info
