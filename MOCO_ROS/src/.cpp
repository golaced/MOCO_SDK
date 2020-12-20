#include <termios.h>  
#include <signal.h>  
#include <math.h>  
#include <stdio.h>  
#include <stdlib.h>  
#include <sys/poll.h>  
 
#include <boost/thread/thread.hpp>  
#include <ros/ros.h>  
#include <geometry_msgs/Twist.h>  
#include <std_msgs/Empty.h>
#define KEYCODE_W 0x77  
#define KEYCODE_A 0x61  
#define KEYCODE_S 0x73  
#define KEYCODE_D 0x64  
 
#define KEYCODE_A_CAP 0x41  
#define KEYCODE_D_CAP 0x44  
#define KEYCODE_S_CAP 0x53  
#define KEYCODE_W_CAP 0x57  
#define KEY_KONGGE 0x20
#define KEY_P 0x70 
using std::cout;
using std::endl;
using namespace std;
class SmartCarKeyboardTeleopNode  
{  
    private:  
        double walk_vel_;  
        double run_vel_;  
        double yaw_rate_;  
        double yaw_rate_run_;  
 
        geometry_msgs::Twist cmdvel_;  
        ros::NodeHandle n_;  
        ros::Publisher pub_,pub_take_off,pub_land,pub_save,pub_stop_save;  

 
    public:  
        SmartCarKeyboardTeleopNode()  
        {  
            pub_ = n_.advertise<geometry_msgs::Twist>("oldx/cmd_vel", 1);  
 	    pub_take_off = n_.advertise<std_msgs::Empty>("oldx/unlock", 1);  
	    pub_land = n_.advertise<std_msgs::Empty>("oldx/lock", 1); 
	    pub_save = n_.advertise<std_msgs::Empty>("txt_save", 1);  
	    pub_stop_save = n_.advertise<std_msgs::Empty>("stop_txt_save", 1);  
            ros::NodeHandle n_private("~");  
            n_private.param("walk_vel", walk_vel_, 0.5);  
            n_private.param("run_vel", run_vel_, 1.0);  
            n_private.param("yaw_rate", yaw_rate_, 1.0);  
            n_private.param("yaw_rate_run", yaw_rate_run_, 1.5);  
        }  
 
        ~SmartCarKeyboardTeleopNode() { }  
        void keyboardLoop();  
 
        void stopRobot()  
        {  
            cmdvel_.linear.x =  cmdvel_.linear.y = cmdvel_.linear.z =0.0;  
            cmdvel_.angular.z = 0.0;  
            pub_.publish(cmdvel_);  
        }  
};  
 
SmartCarKeyboardTeleopNode* tbk;  
int kfd = 0;  
struct termios cooked, raw;  
bool done;  
 
int main(int argc, char** argv)  
{  
    ros::init(argc,argv,"tbk", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);  
    SmartCarKeyboardTeleopNode tbk;  
 
    boost::thread t = boost::thread(boost::bind(&SmartCarKeyboardTeleopNode::keyboardLoop, &tbk));  
 
    ros::spin();  
 
    t.interrupt();  
    t.join();  
    tbk.stopRobot();  
    tcsetattr(kfd, TCSANOW, &cooked);  
 
    return(0);  
}  
 
void SmartCarKeyboardTeleopNode::keyboardLoop()  
{  
    char c;  
    double max_tv = walk_vel_;  
    double max_rv = yaw_rate_;  
    std_msgs::Empty empty1;
    bool dirty = false;  
    int speed[3] = {0};  
    int turn = 0;  
    static int take_flag,save_flag;
    // get the console in raw mode  
    tcgetattr(kfd, &cooked);  
    memcpy(&raw, &cooked, sizeof(struct termios));  
    raw.c_lflag &=~ (ICANON | ECHO);  
    raw.c_cc[VEOL] = 1;  
    raw.c_cc[VEOF] = 2;  
    tcsetattr(kfd, TCSANOW, &raw);  
 
    puts("Reading from keyboard");  
    puts("Use WASD keys to control the robot");  
    puts("Press Shift to move faster");  
 
    struct pollfd ufd;  
    ufd.fd = kfd;  
    ufd.events = POLLIN;  
 
    for(;;)  
    {  
        boost::this_thread::interruption_point();  
 
        // get the next event from the keyboard  
        int num;  
 
        if ((num = poll(&ufd, 1, 250)) < 0)  
        {  
            perror("poll():");  
            return;  
        }  
        else if(num > 0)  
        {  
            if(read(kfd, &c, 1) < 0)  
            {  
                perror("read():");  
                return;  
            }  
        }  
        else  
        {  
            if (dirty == true)  
            {  
                stopRobot();  
                dirty = false;  
            }  
 
            continue;  
        }  
         
  speed[0] = speed[1] =speed[2] =0;  turn = 0;  
        switch(c)  
        {  
            case KEYCODE_W:  
                max_tv = walk_vel_;  
                speed[0] = 1;  
                turn = 0;  
                dirty = true;  
                break;  
            case KEYCODE_S:  
                max_tv = walk_vel_;  
                speed[0] = -1;  
                turn = 0;  
                dirty = true;  
                break;  
            case KEYCODE_A:  
                   max_tv = walk_vel_;  
                speed[1] = -1;  
                turn = 0;  
                dirty = true;  
                break;  
            case KEYCODE_D:  
                        max_tv = walk_vel_;  
                speed[1] = 1;  
                turn = 0;  
                dirty = true;  
                break;  
 
            case KEYCODE_W_CAP:  
                max_tv = walk_vel_;  
                speed[2] = 1;  
                turn = 0;  
                dirty = true;  
                break;  
            case KEYCODE_S_CAP:  
                max_tv = walk_vel_;  
                speed[2] = -1;  
                turn = 0;  
                dirty = true;  
                break;  
            case KEYCODE_A_CAP:  
                max_rv = yaw_rate_run_;  
                speed[0] = speed[1] =speed[2] =0;
                turn = -1;  
                dirty = true;  
                break;  
            case KEYCODE_D_CAP:  
                max_rv = yaw_rate_run_;  
                speed[0] = speed[1] =speed[2] =0;
                turn = 1;  
                dirty = true;  
                break;
	    case KEY_KONGGE:
	
 		take_flag=!take_flag;
		if(take_flag)
		  pub_take_off.publish(empty1);  
		else
		  pub_land.publish(empty1);  
	
	    break;       
   	    case KEY_P:
	
 		save_flag=!save_flag;	
		if(save_flag)
		  pub_save.publish(empty1);  
		else
		  pub_stop_save.publish(empty1); 
		//cout<<"P_key"<<save_flag<<endl;
	    break;               
            default:  
                max_tv = walk_vel_;  
                max_rv = yaw_rate_;  
                speed[0] = speed[1] =speed[2] =0;  
                turn = 0;  
                dirty = false;  
	    break;
        }  
	cmdvel_.linear.x = speed[0] * max_tv*4.5;  
	cmdvel_.linear.y = -speed[1] * max_tv*4.5;  
	cmdvel_.linear.z = speed[2] * max_tv*3.5;  
        cmdvel_.angular.z = -turn * max_rv*2;  
        pub_.publish(cmdvel_);  

  
    }  
}