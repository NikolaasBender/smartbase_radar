#!/usr/bin/python3

import rospy
from track import *
from target_manager import *

from smartbase_radar.msg import RadarTargetArray
from radar_driver.msg import RadarTracks


TM = TargetManager()

# callback
def callback(data):
    array = []
    for track in data.tracks:
        new_track = Track(track)
        array.append(new_track)
    
    TM.update(array)


# listener for radar data   
def listener(topic):
    rospy.Subscriber(topic, RadarTracks, callback)


def main():
    print("started")

    rospy.init_node('radar_node', anonymous = True)
    topics = ['/RadarFrontCenter']

    for topic in topics:
        listener(topic)

    # keeps python moving until data is used
    rospy.spin()



# RUN MAIN
main()