from setuptools import setup, find_packages

package_name = 'fashion_mnist_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cyy',
    maintainer_email='cyy@example.com',
    description='Publish Fashion MNIST recognition result and confidence',
    license='Apache-2.0',
    tests_require=['pytest'],
    # 节点入口点配置
    entry_points={
        'console_scripts': [
            'publisher = fashion_mnist_ros2.publisher_node:main',
            'subscriber = fashion_mnist_ros2.subscriber_node:main',
        ],
    },
)


