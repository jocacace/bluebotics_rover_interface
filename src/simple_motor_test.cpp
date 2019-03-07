#include "ros/ros.h"
#include "libRover.h"


using namespace std;


int main( int argc, char** argv ) {


  ros::init( argc, argv, "rover_interface");



  //Just a simple test!
  Rover rover(false);
  int retCode = rover.init();
  if (retCode != 0) {
    ROS_ERROR("Failed Initialize the Rover");
  }
  
  //cout << "Try to enable tracks: ... " << endl;
  rover.enableTracks();
  //cout << "Enabled?" << endl;
  //sleep(1);
  ros::NodeHandle nh;
  int cycle = 0;
  ros::Rate r(10);
  Time timestamp;
  float v = 0.0;
	float vth = 0.0;
  while (cycle++ < 100 ) {

    retCode = rover.getSpeedVO(timestamp, v, vth);
    if (retCode != 0) {
      // Failed to read the rover speed
      ROS_ERROR("Failed to read the rover speed");
    }
    else printf("[Cycle: %d]: in getSpeedV0: %f - %f\n", cycle,  v, vth);

    retCode = rover.setSpeedVO (0.01, vth);
    if (retCode != 0) {
      // Failed to read the rover speed
      ROS_ERROR("Failed to read the rover speed");
    }
    



    r.sleep();
  }

  retCode = rover.setSpeedVO (-0.0, vth);
  if (retCode != 0) {
    // Failed to read the rover speed
    ROS_ERROR("Failed to read the rover speed");
  }

  return 0;

}
