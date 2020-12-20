# MOCO_SDK
MOCO通用四足机器人控制器ROS SDK

GAIT CMD Define：
#define GAIT_IDLE 0
#define GAIT_TROT 1
#define GAIT_FTROT 2
#define GAIT_WALK 3
#define GAIT_ST_RC 4
#define GAIT_ST_IMU 5
#define GAIT_ST_PUSH 6
#define GAIT_RECOVER 7
#define GAIT_FALL 8
#define GAIT_POWER_OFF 99

Gait FEEDBACK Define:
#define  M_SAFE 0
#define M_STAND_RC 1
#define M_STAND_IMU 2
#define M_STAND_PUSH 3
#define M_TROT 4
#define M_F_TROT 5
#define M_WALK 6
#define M_RECOVER 7
#define M_FALLING 8
#define M_CLIMB 9
#define M_WALKE 10
#define M_CRAWL 11
#define M_PRONK 12
#define M_BOUND 13

run:
roslaunch moco_ros moco_ros.launch

send cmd/vel:
rostopic pub -r 1 moco/cmd_vel geometry_msgs/Twist '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}'

active gait:
rostopic pub -1 moco/cmd_gait std_msgs/Float32 -- 4  
rostopic pub -1 moco/cmd_gait std_msgs/Float32 -- 1

power off:
rostopic pub -1 moco/cmd_gait std_msgs/Float32 -- 99

check attitude:
rostopic pub -1 moco/cmd_gait std_msgs/Float32 -- 4  
