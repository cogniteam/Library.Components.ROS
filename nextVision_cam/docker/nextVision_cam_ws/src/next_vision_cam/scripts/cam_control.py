import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from pymavlink import mavutil

class camControl():
    def __init__(self):
                self.port = rospy.get_param("port")
                self.ip = rospy.get_param("ip")
                self.device = 'udpin:'+str(self.ip)+':'+str(self.port)
                rospy.loginfo(self.device)
                # Start a connection listening to a UDP port
                self.the_connection = mavutil.mavlink_connection(str(self.device))

                # Wait for the first heartbeat
                #   This sets the system and component ID of remote system for the link
                self.the_connection.wait_heartbeat()
                rospy.loginfo("Heartbeat from system (system %u component %u)" %
                            (self.the_connection.target_system, self.the_connection.target_component))
                rospy.Subscriber('/next_vision_cam/cmd_vel', Twist, self.move)

    def move(self,data:Twist):
        pitch = data.linear.x
        roll = data.angular.z
        roll = roll%1 if roll>0 else roll%(-1)
        
        zoom = 3
        self.the_connection.mav.command_long_send(self.the_connection.target_system, self.the_connection.target_component,
                                        mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL, 0, 6, roll, pitch, zoom, 0, 0, 0)


def main():
    rospy.init_node('camControl', anonymous=True)
    camControl()
    rospy.spin()

if __name__ == '__main__':
    main()