import os
import cv2
from ultralytics import YOLO

# ===================== 固定配置（与数据集一致） =====================
CLASS_NAMES = [
    "T-shirt/top", "Trouser", "Pullover", "Dress", "Coat",
    "Sandal", "Shirt", "Sneaker", "Bag", "Ankle boot"
]

# ===================== YOLOv8 推理类封装 =====================
class YOLOv8FashionInfer:
    """Fashion MNIST YOLOv8 推理工具类，提供单张图片推理功能"""
    def __init__(self, model_path):
        """
        初始化推理工具
        :param model_path: 训练好的 YOLOv8 模型权重路径（.pt 文件）
        """
        # 1. 验证模型文件是否存在
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"❌ 模型文件不存在：{model_path}")
        
        # 2. 加载 YOLOv8 模型
        self.model = YOLO(model_path)
        self.model_path = model_path
        print(f"✅ 成功加载 YOLOv8 模型：{model_path}")
    
    def infer_single_image(self, img_input):
        """
        单张图片推理，返回类别名称和置信度
        :param img_input: 图片输入（支持 2 种格式：① 图片文件路径 ② numpy 数组（灰度图/彩色图））
        :return: (cls_name: 类别名称, confidence: 置信度（0-1，保留4位小数）)
        """
        # 1. 加载图片
        if isinstance(img_input, str):
            # 输入为图片路径，读取为灰度图
            img = cv2.imread(img_input, cv2.IMREAD_GRAYSCALE)
            if img is None:
                raise ValueError(f"❌ 无法读取图片文件：{img_input}")
        else:
            # 输入为 numpy 数组，直接使用
            img = img_input
        
        # 2. 执行 YOLOv8 推理
        results = self.model(img, imgsz=64, verbose=False)  # verbose=False 关闭推理日志
        result = results[0]  # 提取第一张图片的推理结果
        
        # 3. 解析推理结果
        if len(result.boxes) > 0:
            # 提取置信度最高的目标（单目标场景，直接取第一个）
            cls_idx = int(result.boxes.cls[0].item())
            confidence = result.boxes.conf[0].item()
            cls_name = CLASS_NAMES[cls_idx]
            return cls_name, round(confidence, 4)
        else:
            # 无目标检测到
            return "Unknown", 0.0
