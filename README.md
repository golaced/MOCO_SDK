<div align=center><img width="600" height="130" src="https://github.com/golaced/OLDX_DRONE_SIM/blob/rmd/support_file/img_file/logo.JPG"/></div>
<div align=center><img width="400" height="300" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/fc2.jpg"/></div>
MOCO通用四足机器人控制器ROS SDK

GAIT CMD Define： <br>
#define GAIT_IDLE 0 <br>
#define GAIT_TROT 1 <br>
#define GAIT_FTROT 2 <br>
#define GAIT_WALK 3 <br>
#define GAIT_ST_RC 4 <br>
#define GAIT_ST_IMU 5 <br>
#define GAIT_ST_PUSH 6 <br>
#define GAIT_RECOVER 7 <br>
#define GAIT_FALL 8 <br>
#define GAIT_POWER_OFF 99 <br>

Gait FEEDBACK Define: <br>
#define  M_SAFE 0 <br>
#define M_STAND_RC 1 <br>
#define M_STAND_IMU 2 <br>
#define M_STAND_PUSH 3 <br>
#define M_TROT 4 <br>
#define M_F_TROT 5 <br>
#define M_WALK 6 <br>
#define M_RECOVER 7 <br>
#define M_FALLING 8 <br>
#define M_CLIMB 9 <br>
#define M_WALKE 10 <br>
#define M_CRAWL 11 <br>
#define M_PRONK 12 <br>
#define M_BOUND 13 <br>

run: <br>
roslaunch moco_ros moco_ros.launch <br>

send cmd/vel: <br>
rostopic pub -r 1 moco/cmd_vel geometry_msgs/Twist '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}' <br>

active gait: <br>
rostopic pub -1 moco/cmd_gait std_msgs/Float32 -- 4   <br>
rostopic pub -1 moco/cmd_gait std_msgs/Float32 -- 1 <br>

power off: <br>
rostopic pub -1 moco/cmd_gait std_msgs/Float32 -- 99 <br>

check attitude: <br>
rostopic pub -1 moco/cmd_gait std_msgs/Float32 -- 4   <br>


1.硬件连接<br>
将树莓派与主控制器连接，参照树莓派引脚定义使用其串口与主控的UART1连接（TX接RX，RX接TX），另外树莓派由于其功耗较大板载5V降压无法带动，需要使用外部供电5V 3A。

2.MOCO_SDK编译<br>
	在完成硬件链接后在Github上下载MOCO_SDK项目，需在树莓派和PC机上建立工作空间完成编译。为方便算法开发树莓派端主要完成对传感器采集并接受控制命令下发给主控制器，而相关感知、图像识别算法则推荐运行在PC机上，编译过程如下：<br>
（1）树莓派端：可仅保留moco_cap_pub相机发布cpp，编译需安装OpenCV和serial库；<br>
（2）PC端：不需要编译MOCO_ROS文件夹，编译需安装OpenCV和ArUco库；<br>

3.ROS测试流程<br>
首先可以选择使用Putty或VNC登录树莓派启动主节点（当然更推荐的方式是在PC端建立无线网络实现远程连接并使用SSH启动具体流程可以参考https://zhuanlan.zhihu.com/p/338576271 ），启动步骤如下：<br>
（1）在catkin_make完成后source当前工作空间，输入roslaunch moco_ros moco_all.launch启动主节点；
注：主任务启动后树莓派会通过串口与控制器交互，采集底层机器人的关节角度、姿态、里程计数据，并发布对应采集的摄像头数据（具体启动是USB还是CSI并口相机可在moco_cv.launch中调节对应的ID号 <param name="camera_name" type="int" value="0" />）<br>
（2）在启动主任务后，如PC机与树莓派实现连接，并按教程配置好主从机IP则可以通过rostopic list命令查看是否能在PC机收到树莓派发布的机载传感器数据，此时可以通过运行moco_cap_sub节点订阅显示相机数据，通过echo命令打印对应的传感器数据；<br>
（3）图像识别测试，在PC机运行roslaunch moco_cv  moco_cv_color.launch启动小球颜色识别程序（默认黄色），通过拖动HSV阈值可改变识别目标，在结果中查看识别效果；<br>
（4）图像识别测试，在PC机运行roslaunch moco_cv  moco_cv_aruco.launch启动二维码（打印项目中对应二维码图片），在结果中查看识别效果；<br>
（5）像素跟踪测试，像素跟踪测试实现对任意图像目标像素识别位置的跟踪，通过控制机器人前后位置和角度保证识别目标在图像中心并保证固定距离，可通过在启动对应识别CV程序后启动moco_track_pix节点跟踪，或者直接启动roslaunch moco_plan  moco_track_pix_color.launch同时启动识别和跟踪程序；<br>
注：启动跟踪程序后会出现对应控制台，通过将gait_mode从0拖到1实现机器人的自动站立，拖到2进入步态控制，使用拖动条可以控制移动速度，拖到en_track开始自动跟踪目标<br>

# 捐赠与项目后续开发计划
____团队计划后期推出5kg~10kg级的足式机器人开发底盘，支持RPlidar激光雷达导航进行SLAM算法验证，能以相同的价格替代目前市面上同类的四轮小车平台如Autolabor等。
 <div align=center><img width="800" height="300" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/r1.jpg"/></div>
____如果您觉得该项目对您有帮助，也为了更好的项目推进和软硬件更新，如果愿意请通过微信捐赠该项目！
<div align=center><img width="440" height="300" src="https://github.com/golaced/OLDX_DRONE_SIM/blob/master/support_file/img_file/pay.png"/></div>

