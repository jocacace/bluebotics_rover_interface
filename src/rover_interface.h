#include "ros/ros.h"
#include "boost/thread.hpp"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "libRover.h"
#include "TooN/TooN.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>

using namespace std;
using namespace TooN;

class rover_ctrl_interface {
    public:
        rover_ctrl_interface();
        void run();
        void c_vel_cb ( geometry_msgs::Twist c_vel );
				void imu_cb ( const sensor_msgs::ImuConstPtr imu);
        void rc_cb( sensor_msgs::Joy joy );
        void vel_from_joy();
        void rover_ctrl();

    private:

        ros::NodeHandle _nh;
        ros::Subscriber _cmd_vel_sub;
        ros::Subscriber _joy_sub;
        ros::Publisher _bat_pub;
        ros::Publisher _odom_pub;
				ros::Subscriber _imu_sub;
				ros::Publisher _body_vel_pub;

        bool _mushroom_joy; // ;)
        bool _control_motors;
				bool _publish_tf;
				bool _rotation_from_imu;
        int _ctrl_rate;
        int _lin_vel_joy_axes;
        int _ang_vel_joy_axes;
        double _max_lin_vel;
        double _max_ang_vel;
				bool _first_joy;

        int _mush_joy_but;
				int _auto_joy_but;
        float _lin_vel;
        float _ang_vel;
        float _joy_lin_vel;
        float _joy_ang_vel;

				float _auto_lin_vel;
				float _auto_ang_vel;
        
        bool _cmd_enabled;
        bool _mush_pressed;
				bool _auto_pressed;
				float _imu_vth;
        
        Rover *_rover;        



};
