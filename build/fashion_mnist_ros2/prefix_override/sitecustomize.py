import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/cyy/fashion_mnist_yolov8_ros2/install/fashion_mnist_ros2'
