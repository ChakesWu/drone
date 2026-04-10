From :drone\DroneWS\drone_cartographer_control.py
can:
1:can lisening current address
2:auto fly (guided mode)

why can't fly:

这个脚本假设飞控已经通过某种方式获得了可靠的本地位置（local_position/pose），但实际：
1: Cartographer 只发布了 TF，没有把定位结果喂给飞控 EKF
如果飞控没有 vision/odometry 输入，它的 local_position 可能是空的或漂移的
