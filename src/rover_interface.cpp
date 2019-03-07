#include "rover_interface.h"

using namespace std;
using namespace TooN;

//---Get parameters
void load_param( double & p, double def, string name ) {
	ros::NodeHandle n_param("~");
	if( !n_param.getParam( name, p))
		p = def;
	cout << name << ": " << "\t" << p << endl;
}

void load_param( int & p, int def, string name ) {
	ros::NodeHandle n_param("~");
	if( !n_param.getParam( name, p))
		p = def;
	cout << name << ": " << "\t" << p << endl;
}

void load_param( bool & p, bool def, string name ) {
	ros::NodeHandle n_param("~");
	if( !n_param.getParam( name, p))
		p = def;
	cout << name << ": " << "\t" << p << endl;
}

void load_param( string & p, string def, string name ) {
	ros::NodeHandle n_param("~");
	if( !n_param.getParam( name, p))
		p = def;
	cout << name << ": " << "\t" << p << endl;
}
//---

rover_ctrl_interface::rover_ctrl_interface() {
  

  load_param (_mushroom_joy, true, "mushroom_joy" );
  load_param (_ctrl_rate, 100, "ctrl_rate");
	load_param (_auto_joy_but, 5, "auto_joy_but");

  load_param(_lin_vel_joy_axes, 1, "lin_vel_joy_axes" );
  load_param(_ang_vel_joy_axes, 3, "ang_vel_joy_axes" );
  load_param(_mush_joy_but, 4, "mush_joy_but" );
  load_param(_control_motors, false, "control_motors" );

  load_param(_max_lin_vel, 0.1, "max_lin_vel");
  load_param(_max_ang_vel, 0.4, "max_ang_vel");
  
  _cmd_vel_sub = _nh.subscribe("/rover/cmd_vel", 0, &rover_ctrl_interface::c_vel_cb, this );
  _joy_sub = _nh.subscribe("/joy", 0, &rover_ctrl_interface::rc_cb, this );
  _odom_pub = _nh.advertise<nav_msgs::Odometry>("odom", 0);
  _bat_pub = _nh.advertise< std_msgs::Float32 > ("/rover/battery", 0);

  _lin_vel = 0.0;
  _ang_vel = 0.0;
  _joy_lin_vel = 0.0;
  _joy_ang_vel = 0.0;
	_auto_lin_vel = 0.0;
	_auto_ang_vel = 0.0;

  _cmd_enabled = false;
	_first_joy = false;
	_auto_pressed = false;

  _rover = new Rover(false);
        
}


void rover_ctrl_interface::c_vel_cb ( geometry_msgs::Twist c_vel ) {
	_auto_lin_vel = c_vel.linear.x;
	_auto_ang_vel = c_vel.angular.z;
	
}

void rover_ctrl_interface::rc_cb( sensor_msgs::Joy joy ) {

  _joy_lin_vel = joy.axes[_lin_vel_joy_axes];
  _joy_ang_vel = joy.axes[_ang_vel_joy_axes];
  _mush_pressed = (joy.buttons[_mush_joy_but] == 1) ? true : false;
	_auto_pressed = (joy.buttons[_auto_joy_but] == 1) ? true : false;

	_first_joy = true;
}

void rover_ctrl_interface::rover_ctrl() {

  ros::Rate r( _ctrl_rate );

  float dt = 1.0/float(_ctrl_rate);
  nav_msgs::Odometry odom;
  ros::Time current_time;

  if ( _rover->init() != 0 ) {
    ROS_ERROR("Failed Initialize the Rover");
    exit(0);
  }
  else 
    ROS_INFO("Rover succesfully initialized");

  if( _control_motors )
    _rover->enableTracks();

  Time timestamp;
  float v = 0.0;
  float vth = 0.0;
	std_msgs::Float32 v_volt;
  float vx, vy;
  float x, y, th;
  vx = vy = 0.0;
  x = y = th = 0.0;

  tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::TransformStamped odom_trans;
  geometry_msgs::Quaternion odom_quat;

  while( ros::ok() ) {
    
    current_time = ros::Time::now();

    //---Get rover state
    if( _rover->getSpeedVO(timestamp, v, vth) ) {
      // Failed to read the rover speed
      ROS_ERROR("Failed to read the rover speed");
      exit(0);
    }
		//vth = -vth;
    
    
    //---Odometry
    th += vth* dt;
    
    vy = v * sin(th);    
    vx = v * cos(th);
    
    x += vx * dt;
    y += vy * dt;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_quat = tf::createQuaternionMsgFromYaw(th);

    odom_trans.transform.rotation = odom_quat;
    //send the transform
    odom_broadcaster.sendTransform(odom_trans);
    //---

		//cout << "vth: " << vth << endl;
		//2cout << "Rover position: " << x << ", " << y << " / " << th << endl;

		if ( _rover->readVoltage(EDIFrontRightTrack, timestamp, v_volt.data) != 0 ) {
    	ROS_ERROR("Failed to Read Battery Voltage, Try to Restart the Rover");
  	  exit(0);	
    }
    //---



    //velocity saturation
		if ( !_auto_pressed ) {
		  _lin_vel = (fabs(_joy_lin_vel) <= _max_lin_vel ) ? _joy_lin_vel : (_joy_lin_vel < 0.0 ) ? -_max_lin_vel : _max_lin_vel;
		  _ang_vel = (fabs(_joy_ang_vel) <= _max_ang_vel ) ? _joy_ang_vel : (_joy_ang_vel < 0.0 ) ? -_max_ang_vel : _max_ang_vel;
		}
		else {
			cout << "Vel pre saturation: " << _auto_lin_vel << " - " << _auto_ang_vel << endl;
		  _lin_vel = (fabs(_auto_lin_vel) <= _max_lin_vel ) ? _auto_lin_vel : (_auto_lin_vel < 0.0 ) ? -_max_lin_vel : _max_lin_vel;
		  _ang_vel = (fabs(_auto_ang_vel) <= _max_ang_vel ) ? _auto_ang_vel : (_auto_ang_vel < 0.0 ) ? -_max_ang_vel : _max_ang_vel;
		}


    if( _control_motors ) {
      if( !_mush_pressed ) {
        _lin_vel = _ang_vel = 0.0;
      }

      if (_rover->setSpeedVO ( _lin_vel, _ang_vel) != 0) {
        ROS_ERROR("Failed to read the rover speed");
        exit(0);
      }
    }

    _bat_pub.publish( v_volt );
    
    r.sleep();
  }
}

void rover_ctrl_interface::run() {

  boost::thread rover_ctrl_t( &rover_ctrl_interface::rover_ctrl, this );
  ros::spin();
}

int main( int argc, char** argv ) {


  ros::init( argc, argv, "rover_interface");
  rover_ctrl_interface rci;
  rci.run();

  return 0;

}
