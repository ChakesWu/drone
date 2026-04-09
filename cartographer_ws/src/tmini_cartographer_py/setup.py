from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tmini_cartographer_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装launch文件（修复路径写法）
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.lua')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='drone',
    maintainer_email='drone@todo.todo',
    description='T-MINI雷达 + Cartographer建图节点',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    # 节点入口配置
    entry_points={
        'console_scripts': [
            'tmini_cartographer_node = tmini_cartographer_py.tmini_cartographer_node:main'
        ],
    },
)
