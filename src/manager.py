#!/usr/bin/python

import rospy
from track import *
from target_manager import *

from smartbase_radar.msg import RadarTargetArray


TM = TargetManager()

# callback
def callback(data):
    array = []
    rospy.loginfo(len(data.targets))
    for track in data.targets:
        # print(target)
        new_track = Track(track)
        array.append(new_track)
    
    TM.update(array)


# listener for radar data   
def listener(topic):
    rospy.Subscriber(topic, RadarTargetArray, callback)


def main():
    print("started")

    rospy.init_node('listener', anonymous = True)
    
    # got topic from rosBAG -- this is the raw can bus data

    topics = ["/srr2_front/targets_left", "/srr2_front/targets_right"]

    for topic in topics:
        listener(topic)

    # keeps python moving until data is used
    rospy.spin()



# RUN MAIN
main()