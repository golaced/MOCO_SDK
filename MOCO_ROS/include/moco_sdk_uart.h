#include <iostream>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <string.h>
#include <unistd.h>


#define BUFF_SIZE 200

#define D_LEG 0
#define X_LEG 1
#define T_LEG 2
#define L 0
#define R 1
#define F 0
#define B 1
#define FL 0
#define BL 1
#define FL1 0
#define BL1 1
#define FL2 2
#define BL2 3
#define Xr 0
#define Yr 1
#define Zr 2
#define P 0
#define I 2
#define D 1
#define FB 3
#define IP 4
#define II 5
#define ID 6
#define PITr 0
#define ROLr 1
#define YAWr 2
#define YAWrr 3

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

typedef struct {
float W;
float H;
float L1;
float L2;
float L3;
char rc_mode;
char robot_mode;
char imu_mode;

}_ROBOT_P;

typedef struct {
float bat;
float rc_weight;
float rc_spd_cmd[3];
float rc_pos_cmd[3];
float rc_att_rate_cmd[3];
}_SYS;

typedef struct {
char connect;
int loss_cnt;

float att[3];
float att_rate[3];

float pos_n[3];
float spd_n[3];
float spd_b[3];

float acc_n[3];
float acc_b[3];

float q[4][3];
float epos_n[4][3];
float epos_b[4][3];
char td_state[4];

float cog_nn[3];
float zmp_nn[3];
float support_cog_nn[3];
float epos_nn[4][3];

_ROBOT_P param;
_SYS system;
}_ROBOT;

extern _ROBOT moco;

void rx_anal(unsigned char com_data);

//-----------------------------------------------------------------
typedef struct {
char mode;
char mode_ros;
char power_off;
char gait_mode;
float rc_spd_cmd[3];
float rc_pos_cmd[3];
float rc_att_cmd[3];
float rc_att_rate_cmd[3];
}_SDK;

extern _SDK moco_sdk;

extern char SendBuff_USB[BUFF_SIZE];
extern int usb_send_cnt;

int uart_formate_ocu_spd_remote(void);
int uart_formate_power_off(void);
void sdk_init(void);
void sdk_reset(void);
int sdk_thread(void);

void set_pos_z(float set);
void set_spd_x(float set);
void set_spd_y(float set);
void set_spd_yaw_rate(float set);
void set_att_pit(float set);
void set_att_rol(float set);
void set_att_yaw(float set);
