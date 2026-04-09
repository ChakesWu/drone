import os
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # ====================== 1. 定义路径和参数 ======================
    # YDLIDAR 驱动包路径
    ydlidar_share_dir = get_package_share_directory('ydlidar_ros2_driver')
    # Cartographer 配置路径
    cartographer_share_dir = get_package_share_directory('cartographer_ros')
    # 自定义包路径（可选）
    tmini_share_dir = get_package_share_directory('tmini_cartographer_py')

    # 声明雷达参数文件参数（默认使用 Tmini_12m.yaml）
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(ydlidar_share_dir, 'params', 'Tmini_12m.yaml'),
        description='Path to the YDLIDAR ROS2 parameters file'
    )
    parameter_file = LaunchConfiguration('params_file')

    # ====================== 2. YDLIDAR 核心节点（替代原 Include 方式） ======================
    # 雷达驱动节点（生命周期节点，适配官方配置）
    ydlidar_driver_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[parameter_file],
        namespace='/',
    )

    # 激光坐标系 TF 转换（关键：base_link -> laser_frame，匹配 Cartographer 期望的坐标系）
    laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0', '0', '0.02', '0', '0', '0', '1', 'base_link', 'laser_frame'],
        output='screen'
    )

    # ====================== 3. 自定义接收节点 ======================
    tmini_cartographer_node = Node(
        package='tmini_cartographer_py',
        executable='tmini_cartographer_node',
        name='tmini_node',
        output='screen',
        # 确保自定义节点输出的 /scan 话题与 Cartographer 兼容
        remappings=[
            ('/scan', '/scan'),  # 显式映射，避免话题名冲突
        ]
    )

    # ====================== 4. Cartographer 核心节点 ======================
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_core',
        output='screen',
        arguments=[
            '-configuration_directory', os.path.join(cartographer_share_dir, 'configuration_files'),
            '-configuration_basename', 'backpack_2d.lua'  # 官方 2D 配置
        ],
        # 关键映射：
        # 1. 订阅雷达的 /scan 话题（laser_frame 坐标系）
        # 2. 输出的地图话题为 /map
        remappings=[
            ('/scan', '/scan'),                # 明确订阅雷达扫描话题
            ('/tf', '/tf'),                    # 订阅 TF 变换
            ('/tf_static', '/tf_static')       # 订阅静态 TF
        ],
        # 新增参数：解决时间戳和数据丢弃问题
        parameters=[{
            'use_sim_time': False,             # 禁用仿真时间，使用系统时间
            'num_threads': 4                   # 匹配 Ceres 最大线程数，避免警告
        }]
    )

    # ====================== 5. 地图发布节点（生成 /map 话题） ======================
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        arguments=[
            '-resolution', '0.05',            # 地图分辨率 5cm
            '-publish_period_sec', '0.5'      # 地图发布频率 2Hz
        ],
        # 明确发布 /map 话题，供 RViz 订阅
        remappings=[
            ('/map', '/map'),                  # 确保输出 /map 话题
            ('/map_metadata', '/map_metadata')
        ]
    )

    # ====================== 6. RViz2 节点（加载官方雷达配置） ======================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # 使用 YDLIDAR 官方 RViz 配置（已适配 /scan 和 /map 话题）
        arguments=['-d', os.path.join(ydlidar_share_dir, 'config', 'ydlidar.rviz')]
    )

    # ====================== 7. 组装所有节点 ======================
    ld = LaunchDescription()
    
    # 先声明参数
    ld.add_action(params_file_arg)
    
    # 启动顺序：先启动雷达驱动和 TF → 再启动数据处理 → 最后可视化
    ld.add_action(ydlidar_driver_node)    # 雷达驱动（输出 /scan 话题）
    ld.add_action(laser_tf_node)          # 激光 TF 变换
    ld.add_action(tmini_cartographer_node)# 自定义接收节点
    ld.add_action(cartographer_node)      # Cartographer 核心（订阅 /scan）
    ld.add_action(occupancy_grid_node)    # 生成 /map 话题
    ld.add_action(rviz_node)              # RViz（显示 /scan 和 /map）

    return ld