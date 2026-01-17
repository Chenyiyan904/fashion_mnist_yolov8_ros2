# 文件名：publisher_node.py
# 放置路径：~/fashion_mnist_yolov8_ros2/src/fashion_mnist_ros2/fashion_mnist_ros2/
import rclpy
from rclpy.node import Node
import random
from std_msgs.msg import String  # 改用内置字符串消息

class FashionMnistRecogPublisher(Node):
    """Fashion MNIST识别结果发布者，发布分类标签和置信度"""
    def __init__(self):
        super().__init__("fashion_mnist_recog_publisher")
        
        # 创建发布者（发布内置字符串消息）
        self.publisher_ = self.create_publisher(
            String,
            "fashion_mnist_result",
            10
        )
        
        # 配置定时器（1.5秒/次）
        self.timer_period = 1.5
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # Fashion MNIST标准分类标签
        self.fashion_labels = [
            "T-shirt/top", "Trouser", "Pullover", "Dress", "Coat",
            "Sandal", "Shirt", "Sneaker", "Bag", "Ankle boot"
        ]
        
        # 消息计数器
        self.msg_count = 0
        
        # 启动日志
        self.get_logger().info("✅ Fashion MNIST 识别结果发布者已启动")
        self.get_logger().info(f"📌 发布话题：fashion_mnist_result")
        self.get_logger().info(f"⏱  发布间隔：{self.timer_period} 秒")

    def timer_callback(self):
        """构建并发布封装好的识别结果（标签+置信度）"""
        self.msg_count += 1
        
        # 模拟识别结果
        label = random.choice(self.fashion_labels)
        confidence = round(random.uniform(0.85, 0.99), 2)
        
        # 封装到字符串消息中（清晰展示识别信息和正确率）
        msg = String()
        msg.data = f"消息{self.msg_count} | 分类标签：{label} | 识别正确率：{confidence:.2f}（{confidence*100:.1f}%）"
        
        # 发布消息
        self.publisher_.publish(msg)
        
        # 打印发布日志
        self.get_logger().info(f"📤 发布识别结果：{msg.data}")

# 顶层main函数（ROS2入口）
def main(args=None):
    rclpy.init(args=args)
    publisher_node = FashionMnistRecogPublisher()
    
    try:
        rclpy.spin(publisher_node)
    except KeyboardInterrupt:
        publisher_node.get_logger().info("🛑 发布者节点即将退出...")
    finally:
        publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()


