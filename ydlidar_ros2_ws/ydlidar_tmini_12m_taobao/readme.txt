	激光雷逹T-mini Pro 12m(接線usb2.0 下層)
	安装 YDLidar-SDK 及依赖
		测试 YDLidar-SDK
			cd ~/YDLidar-SDK/build
			./tri_test  # 按提示选择：端口选1（/dev/ttyUSB0）、波特率选4（230400）、选yes
	安装 YDLidar ROS2 驱动
		echo "source ~/ydlidar_ros2_ws/install/setup.bash" >> ~/.bashrc
		source ~/.bashrc
		
		LiDAR 节点配置文件
			# 创建并编辑参数文件 Tmini.yaml
				cd ~/ydlidar_ros2_ws/src/ydlidar_ros2_driver/params
				gedit Tmini.yaml  # 打开编辑器后，粘贴文中指定的参数内容并保存退出

			# 创建并编辑启动文件 Tmini_launch.py
				cd ~/ydlidar_ros2_ws/src/ydlidar_ros2_driver/launch
				gedit Tmini_launch.py  # 打开编辑器后，粘贴文中指定的启动文件内容并保存退出
		
		运行启动文件
			ros2 launch ydlidar_ros2_driver Tmini_launch.py
		
		运行 Rviz2 查看雷达数据
			rviz2 -d ~/ydlidar_ros2_ws/src/ydlidar_ros2_driver/config/ydlidar.rviz
		
		检查 ROS2 话题
			# 查看话题列表
			ros2 topic list

			# 查看雷达数据话题（二选一，根据实际情况调整）
			ros2 topic echo /scan
			# 或
			ros2 topic echo /laser_scan


### Launch Lidar

```bash
ros2 launch ydliar_ros2_driver ydlidar_launch.py
```

### RVIZ with Colour

**Note: Just run this don't run the above command as well**  

```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch_view.py
```


Tmini_12m.yaml放入
	ydlidar_ros2_ws\src\ydlidar_ros2_driver\params

Tmini_launch放入
	ydlidar_ros2_ws\src\ydlidar_ros2_driver\launch
	
返回ydlidar_ros2_ws目錄執行
	colcon build --symlink-install
	
修改/home/drone/ydlidar_ros2_ws/install/ydlidar_ros2_driver/share/ydlidar_ros2_driver/launch目錄下
	ydlidar_launch_view.py內的Tmini_12m
		    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'Tmini_12m.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')
	