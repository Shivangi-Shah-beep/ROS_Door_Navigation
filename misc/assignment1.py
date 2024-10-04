import rospy
from geometry_msgs.msg import Twist

def Walk():
  pub=rospy.Publisher('/cmd_vel', Twist,queue_size=10)
  rospy.init_node('Walk',anonymous='True')
  rate=rospy.Rate(10)
  while not rospy.is_shutdown():
    move_cmd=Twist()
    move_cmd.linear.x=1
    move_cmd.angular.z=0.5
    rospy.loginfo(move_cmd)
    pub.publish(move_cmd)
    rate.sleep()

if __name__=='__main__':
  try:
    Walk()
  except rospy.ROSInterruptException:
    pass

   
