import os
from ultralytics import YOLO

# ===================== 配置参数（自动推导路径，无需手动修改） =====================
# 工程根目录
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# 数据集配置文件路径（src/dataset/fashion_mnist.yaml）
DATASET_CONFIG_PATH = os.path.join(ROOT_DIR, "dataset", "fashion_mnist.yaml")
# 模型权重保存目录（src/weights）
WEIGHTS_SAVE_DIR = os.path.join(ROOT_DIR, "weights")
# 训练参数
EPOCHS = 1 # 训练轮数（Fashion MNIST 简单，50轮足够收敛）
IMG_SIZE = 32  # 输入图片尺寸（放大 28×28 到 64×64，提升训练效果）
BATCH_SIZE = 32  # 批次大小（根据内存调整，CPU 环境建议 16/32）
DEVICE = "cpu"  # 训练设备（无 GPU 填 "cpu"，有 GPU 填 "0"）

# ===================== 主训练函数 =====================
def train_yolov8_fashion_mnist():
    """训练 YOLOv8 模型识别 Fashion MNIST 数据集"""
    # 1. 创建权重保存目录
    os.makedirs(WEIGHTS_SAVE_DIR, exist_ok=True)
    print(f"✅ 模型权重保存目录：{WEIGHTS_SAVE_DIR}")
    
    # 2. 验证数据集配置文件是否存在
    if not os.path.exists(DATASET_CONFIG_PATH):
        raise FileNotFoundError(f"❌ 数据集配置文件不存在，请先运行数据集转换脚本：{DATASET_CONFIG_PATH}")
    
    # 3. 加载 YOLOv8 模型（nano 版，轻量快速，适合小数据集）
    print("=============== 加载 YOLOv8 模型 ===============")
    # 从空配置创建模型（也可加载预训练模型：YOLO("yolov8n.pt")）
    model = YOLO("yolov8n.yaml")
    print("✅ YOLOv8n 模型加载成功")
    
    # 4. 开始训练
    print("=============== 开始训练模型 ===============")
    train_results = model.train(
        data=DATASET_CONFIG_PATH,
        epochs=EPOCHS,
        imgsz=IMG_SIZE,
        batch=BATCH_SIZE,
        device=DEVICE,
        project=WEIGHTS_SAVE_DIR,
        name="fashion_mnist_yolov8n",
        exist_ok=True,
        verbose=True  # 显示详细训练日志
    )
    
    # 5. 验证训练结果
    print("=============== 开始验证模型 ===============")
    val_results = model.val()
    print(f"✅ 验证集 mAP@0.5：{val_results.box.map50:.4f}（值越高，模型效果越好）")
    
    # 6. 保存最优模型
    best_model_path = os.path.join(WEIGHTS_SAVE_DIR, "fashion_mnist_best.pt")
    model.save(best_model_path)
    print(f"✅ 最优模型已保存：{best_model_path}")
    
    # 7. 训练完成提示
    print("=============== 模型训练全部完成 ===============")

# ===================== 主函数（执行入口） =====================
if __name__ == "__main__":
    try:
        train_yolov8_fashion_mnist()
    except Exception as e:
        print(f"❌ 训练过程中出现错误：{e}")
