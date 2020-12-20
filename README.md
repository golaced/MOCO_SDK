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


# 捐赠与项目后续开发计划
____团队计划后期推出5kg~10kg级的足式机器人开发底盘，支持RPlidar激光雷达导航进行SLAM算法验证，能以相同的价格替代目前市面上同类的四轮小车平台如Autolabor等。
 <div align=center><img width="800" height="300" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/r1.jpg"/></div>
____如果您觉得该项目对您有帮助，也为了更好的项目推进和软硬件更新，如果愿意请通过微信捐赠该项目！
<div align=center><img width="440" height="300" src="https://github.com/golaced/OLDX_DRONE_SIM/blob/master/support_file/img_file/pay.png"/></div>

