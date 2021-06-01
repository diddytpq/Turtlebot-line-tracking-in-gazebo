import numpy as np
import time
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

def main():
    #left_wheel = rospy.Publisher('/turtlebot_controller/wheel_L/command', Float64, queue_size=10)
    #right_wheel = rospy.Publisher('/turtlebot_controller/wheel_R/command', Float64, queue_size=10)
    rospy.wait_for_service('gazebo/get_model_state')
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()


    rospy.init_node('turtlebot_control', anonymous=True)

    rate = rospy.Rate(1000) # 100hz


    #left_wheel.publish(0)
    #right_wheel.publish(0)

    vel_msg.linear.x = 0
    vel_msg.angular.z = 0

    velocity_publisher.publish(vel_msg)

    print("send")
    rate.sleep()



if __name__ == '__main__':

    try:
        main()

    except rospy.ROSInterruptException:
       pass