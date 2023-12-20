#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool
import subprocess

class map_saver():

    def __init__(self):
        rospy.loginfo(f'Start save map node')
        self.path = rospy.get_param("path",'/opt/nimbus/data')
        self.map_name = rospy.get_param("map_name",'')

        self.map_save_command = 'rosrun map_server map_saver' if len(self.map_name) == 0 else f'rosrun map_server map_saver -f {self.map_name}'
        self.custom_env = {
            'MAP_SAVE_PATH': f'{self.path}',
            'MAP_SAVE_COMMAND': f'{self.map_save_command}'
        }
        rospy.Subscriber('map_saver/save', Bool, self.saveMap)
    
    def saveMap(self,msg:Bool):
        if msg.data:
            output = subprocess.run('/map_saver_ws/save_map.sh', shell=True, text=True,env=self.custom_env)
            rospy.loginfo(f'saved map in {self.path}')
            rospy.loginfo(f'map server output: \n {output}')

def main():
    rospy.init_node('map_saver_node', anonymous=True)
    ms = map_saver()
    rospy.spin()

if __name__ == '__main__':
    main()
