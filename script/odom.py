#!/usr/bin/env python

import rospy
import roslib
import tf
import math
import std_msgs.msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

 
class get_odom():

    def __init__(self):
        rospy.init_node("odometry_calculation")
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)
        self.rate = rospy.get_param("~rate", 100)


        body_vel_topic = rospy.get_param("~body_vel_topic", "/rover/body_vel")
        imu_topic = rospy.get_param("~imu_topic", "/imu/data")
        gps_topic = rospy.get_param("~gps_topic", "/raw_fix")
        self.use_gps = rospy.get_param("~use_gps", False)
        output_topic = rospy.get_param("~output_topic", "/odom")
        self.publish_tf = rospy.get_param("~publish_tf", True )

        
				#Input
        rospy.Subscriber(body_vel_topic, Twist, self.bodyVelCb)
        rospy.Subscriber(imu_topic, Imu, self.imuCb)
        if ( self.use_gps ):
            rospy.Subscriber(gps_topic, NavSatFix, self.gpsCb)
        #Output
        self.odometry_pub = rospy.Publisher(output_topic, Odometry, queue_size=0) 
        self.yaw = 0.0

        self.imu_ready = False
        self.gps_ready = False
        self.bvel_ready = False

        self.bvel = Twist()
        self.imu = Imu()

    def spin(self):
    
        r = rospy.Rate(self.rate)
    
        vx = 0.0
        vy = 0.0
        dt = 1.0/float(self.rate)
        x = 0.0 
        y = 0.0


        odom = Odometry()
        br = tf.TransformBroadcaster()

        ###### main loop  ######
        while not rospy.is_shutdown():
            angles = tf.transformations.euler_from_quaternion([self.imu.orientation.x, self.imu.orientation.y, self.imu.orientation.z, self.imu.orientation.w])
            yaw = angles[2]
            
            vx = self.bvel.linear.x * math.sin( yaw )
            vy = self.bvel.linear.x * math.cos( yaw )
            
            x = x + vx*dt
            y = y + vy*dt

            odom.header = std_msgs.msg.Header()
            odom.header.stamp = rospy.Time.now() 
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.orientation.w = self.imu.orientation.w           
            odom.pose.pose.orientation.x = self.imu.orientation.x           
            odom.pose.pose.orientation.y = self.imu.orientation.y           
            odom.pose.pose.orientation.z = self.imu.orientation.z      
            odom.twist.twist.linear.x = vx
            odom.twist.twist.linear.y = vy
            odom.twist.twist.angular.z = self.imu.angular_velocity.z

            print yaw
            #print angles
            self.odometry_pub.publish( odom )     
            if ( self.publish_tf == True ):
                br.sendTransform( (x, y, 0), (self.imu.orientation.x, self.imu.orientation.y, self.imu.orientation.z, self.imu.orientation.w) , rospy.Time.now(), "odom", "base_footprint")

            r.sleep()
                
    def imuCb( self, msg ): 

        self.imu = msg

        self.imu_ready = True
               
    def bodyVelCb( self, msg ):
        self.bvel = msg
        self.bvel_ready = True 

    def gpsCb( self, msg ):  
        self.gps_ready = True


if __name__ == '__main__':
    """ main """
    try:
        odom = get_odom()
        odom.spin()
    except rospy.ROSInterruptException:
        pass
