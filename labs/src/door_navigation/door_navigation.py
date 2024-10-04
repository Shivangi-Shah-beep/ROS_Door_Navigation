

#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class Door():
    def __init__(self):
        # Create a publisher for the door torque
        self.door_torque_pub = rospy.Publisher('/hinged_glass_door/torque', Float64, queue_size=10)
        # Ensure publisher is ready
        rospy.sleep(1)

    def open(self, torque):
        # Publish the torque to open the door
        rospy.loginfo(f'Publishing torque {torque} to open the door...')
        self.door_torque_pub.publish(Float64(torque))
        rospy.loginfo('Torque for opening the door has been published.')

    def close(self, torque):
        # Publish negative torque to close the door
        rospy.loginfo(f'Publishing torque {-torque} to close the door...')
        self.door_torque_pub.publish(Float64(-torque))
        rospy.loginfo('Torque for closing the door has been published.')

class Robot():
    def __init__(self):
        self.robot_velocity_pub=rospy.Publisher('/cmd_vel', Twist, queue_size=10)
       
        self.speed = 0.0
        self.timer = None
        self.speed_duration = 10  # Duration to send speed commands (in seconds)
        self.is_moving = False
       

    def walk(self,speed):
        
        self.speed = speed
        rospy.loginfo(f"The speed is {self.speed}")
        move_cmd=Twist()
        move_cmd.linear.x=self.speed
        self.robot_velocity_pub.publish(move_cmd)
        self.is_moving = True
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback, oneshot=False)

    def timer_callback(self, event):
        if self.is_moving:
           move_cmd=Twist()
           move_cmd.linear.x=self.speed
           self.robot_velocity_pub.publish(move_cmd)
           #rospy.loginfo('Moving the robot at {speed}')
 
    def stop(self):
        rospy.loginfo('Stopping the robot')
        stop_cmd=Twist()
        stop_cmd.linear.x=0
        self.robot_velocity_pub.publish(stop_cmd)
        self.is_moving = False
        if self.timer:
            self.timer.shutdown()

 

def main():
    # Initialize the ROS node
    rospy.init_node('door_navigation', anonymous=True)

    # Create an instance of the Door class
    door = Door()
    robot=Robot()
    
    #Move the robot
 
  # Open the door with a specified torque
    #rospy.sleep(8)
    torque_value = 10.0  # Example torque value (you can adjust this)
    rospy.loginfo("Opening the door...")
    door.open(torque_value)

    speed_value=rospy.get_param("~speed_value",2.0)
    robot.walk(speed_value)
    rospy.loginfo("moving the robot")
    rospy.sleep(3)

    rospy.loginfo("Stopping the robot")
    robot.stop()
    rospy.sleep(2)

    rospy.loginfo("Closing the door after delay...")
    door.close(torque_value)
    rospy.sleep(2)



if __name__ == '__main__':
    try:
        # Run the main function
        main()
    except rospy.ROSInterruptException:
        pass

   
