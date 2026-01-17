import os
import numpy as np
from PIL import Image
import random

# ===================== 配置参数（无需修改，自动适配目录） =====================
# 工程根目录（通过当前脚本路径自动推导）
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# 数据集根目录（src/dataset）
DATASET_ROOT = os.path.join(ROOT_DIR, "dataset")
# 训练/验证图片输出目录
TRAIN_OUTPUT = os.path.join(DATASET_ROOT, "train")
VAL_OUTPUT = os.path.join(DATASET_ROOT, "val")
# Fashion MNIST 类别名称（固定）
CLASS_NAMES = [
    "T-shirt/top", "Trouser", "Pullover", "Dress", "Coat",
    "Sandal", "Shirt", "Sneaker", "Bag", "Ankle boot"
]
# 模拟数据集参数（无需下载，直接生成符合格式的灰度图）
# 找到原模拟数据集参数，替换为以下内容
TRAIN_SAMPLE_COUNT = 900  # 训练集900张（核心修改：大幅减少样本量）
VAL_SAMPLE_COUNT = 100     # 验证集100张（核心修改：快速验证）
IMG_SIZE = 32  # 额外优化：图片尺寸从28→32，兼容YOLOv8且更快（可选，也可保留28）


# ===================== 目录创建函数 =====================
def create_dirs():
    """创建所需的所有目录，不存在则自动创建"""
    dir_list = [
        TRAIN_OUTPUT,
        VAL_OUTPUT,
        os.path.join(DATASET_ROOT, "labels/train"),
        os.path.join(DATASET_ROOT, "labels/val")
    ]
    for dir_path in dir_list:
        os.makedirs(dir_path, exist_ok=True)
        print(f"目录已创建/存在：{dir_path}")

# ===================== 模拟生成Fashion MNIST格式图片（跳过torchvision） =====================
def generate_fashion_image():
    """生成28×28灰度图（模拟Fashion MNIST，满足YOLO训练格式要求）"""
    # 生成随机灰度值数组（0-255，模拟手写服装图案）
    img_np = np.random.randint(0, 255, size=(IMG_SIZE, IMG_SIZE), dtype=np.uint8)
    # 增加局部明暗对比，更贴近真实数据集
    for i in range(IMG_SIZE):
        for j in range(IMG_SIZE):
            if (i > 8 and i < 20) and (j > 8 and j < 20):
                img_np[i][j] = img_np[i][j] // 2 + 50
    return img_np

# ===================== 数据集转换函数（核心：无torchvision依赖） =====================
def convert_dataset(train=True):
    """
    生成 YOLOv8 支持的 Fashion MNIST 格式数据集（无需torchvision）
    :param train: True=生成训练集，False=生成验证集
    """
    # 1. 配置输出路径和数据量
    output_img_dir = TRAIN_OUTPUT if train else VAL_OUTPUT
    output_label_dir = os.path.join(DATASET_ROOT, "labels/train") if train else os.path.join(DATASET_ROOT, "labels/val")
    sample_count = TRAIN_SAMPLE_COUNT if train else VAL_SAMPLE_COUNT
    
    # 2. 逐张生成图片+标注文件
    for idx in range(sample_count):
        # 2.1 随机生成类别标签（0-9，对应10种服装）
        cls_label = random.randint(0, 9)
        
        # 2.2 生成并保存28×28灰度图
        img_np = generate_fashion_image()
        img_pil = Image.fromarray(img_np, mode="L")  # 转为PIL灰度图
        img_file_name = f"fashion_{idx:06d}.jpg"
        img_save_path = os.path.join(output_img_dir, img_file_name)
        img_pil.save(img_save_path)
        
        # 2.3 生成 YOLO 格式标注文件（.txt）
        yolo_annotation = f"{cls_label} 0.5 0.5 1.0 1.0\n"
        label_file_name = f"fashion_{idx:06d}.txt"
        label_save_path = os.path.join(output_label_dir, label_file_name)
        with open(label_save_path, "w", encoding="utf-8") as f:
            f.write(yolo_annotation)
    
    # 3. 生成完成提示
    dataset_type = "训练" if train else "验证"
    print(f"✅ {dataset_type}集生成完成，共处理 {sample_count} 张图片")

# ===================== 生成 YOLO 数据集配置文件 =====================
def create_yolo_dataset_config():
    """生成 fashion_mnist.yaml 配置文件，供 YOLOv8 训练使用"""
    config_path = os.path.join(DATASET_ROOT, "fashion_mnist.yaml")
    # 配置文件内容
    config_content = f"""# Fashion MNIST YOLOv8 数据集配置文件
path: {DATASET_ROOT}  # 数据集根目录（绝对路径）
train: train  # 训练图片目录（相对 path）
val: val      # 验证图片目录（相对 path）

# 类别配置（索引与标签对应）
names:
"""
    # 添加类别名称
    for idx, cls_name in enumerate(CLASS_NAMES):
        config_content += f"  {idx}: {cls_name}\n"
    
    # 写入配置文件
    with open(config_path, "w", encoding="utf-8") as f:
        f.write(config_content)
    
    print(f"✅ YOLO 数据集配置文件已生成：{config_path}")

# ===================== 主函数（执行入口） =====================
if __name__ == "__main__":
    print("=============== 开始生成 Fashion MNIST 数据集（跳过torchvision） ===============")
    create_dirs()  # 第一步：创建目录
    convert_dataset(train=True)  # 第二步：生成训练集
    convert_dataset(train=False)  # 第三步：生成验证集
    create_yolo_dataset_config()  # 第四步：生成配置文件
    print("=============== 数据集生成全部完成 ===============")
