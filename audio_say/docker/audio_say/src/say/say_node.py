#!/usr/bin/env python3
import rospy
import sys
from std_msgs.msg import String
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient


class string_say:

    def __init__(self):
        self.sub = rospy.Subscriber("/msg_to_play",String,self.stringCallback, queue_size = 1)
        self.soundhandle = SoundClient()
    
    def say_from_topic(self,sentence):
        
        rospy.sleep(1)
        voice = rospy.get_param("say_node/voice")
        volume = rospy.get_param("say_node/volume")

        s = str(sentence.data)
        rospy.loginfo('Saying: %s' % s)
        rospy.loginfo('Voice: %s' % voice)
        rospy.loginfo('Volume: %s' % volume)

        self.soundhandle.say(s, voice, volume)
        rospy.sleep(1)



    def stringCallback(self,string_msg):
        if string_msg != None:
            self.say_from_topic(string_msg)

if __name__ == '__main__':

    rospy.init_node('say_node', anonymous=True)

    saying_string = string_say()
    
    rospy.spin() 
