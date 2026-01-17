# 文件名：subscriber_node.py
# 放置路径：~/fashion_mnist_yolov8_ros2/src/fashion_mnist_ros2/fashion_mnist_ros2/
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # 改用内置字符串消息，避免自定义msg问题

class FashionMnistRecogSubscriber(Node):
    """Fashion MNIST识别结果订阅者，解析并打印标签和置信度"""
    def __init__(self):
        super().__init__("fashion_mnist_recog_subscriber")
        
        # 创建订阅者（订阅内置字符串消息）
        self.subscription = self.create_subscription(
            String,
            "fashion_mnist_result",
            self.listener_callback,
            10
        )
        self.subscription  # 防止未使用变量警告
        
        # 启动日志
        self.get_logger().info("✅ Fashion MNIST 识别结果订阅者已启动，等待接收消息...")
        self.get_logger().info(f"📌 订阅话题：fashion_mnist_result")

    def listener_callback(self, msg):
        """解析封装好的识别信息，打印标签和置信度"""
        self.get_logger().info(f"📥 接收识别结果：{msg.data}")
        self.get_logger().info("-" * 50)

# 顶层main函数（ROS2入口）
def main(args=None):
    rclpy.init(args=args)
    subscriber_node = FashionMnistRecogSubscriber()
    
    try:
        rclpy.spin(subscriber_node)
    except KeyboardInterrupt:
        subscriber_node.get_logger().info("🛑 订阅者节点即将退出...")
    finally:
        subscriber_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()



