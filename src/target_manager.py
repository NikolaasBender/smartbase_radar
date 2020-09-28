from target import *
import statistics
from visualization_msgs.msg import MarkerArray

# This is for grouping
# import numpy as np
# from sklearn.cluster import MeanShift

# THIS CLASS MANAGES ALL OF THE TARGETS (hl/coord/1)


class TargetManager:
    def __init__(self):
        # super().__init__()
        # a list of active targets
        self.active_targets = []
        self.viz_targets = rospy.Publisher('targets_array_viz', MarkerArray)
        self.marker_array = MarkerArray()

    # take in raw tracks and add to existing target or create new target
    # more of a callback
    def update(self, tracks):
        return

    # This creates the selection criteria for divvying up the tracks for tracks
    def selection_criteria(self, sorted_tracks):
        return


    # This looks for groups of targets
    # USE DISTANCE AND VELOCITY TO GROUP
    def grouper(self, tracks):
        return

    # This looks for groups of targets
    # USE DISTANCE AND VELOCITY TO GROUP
    def pos_grouper(self, tracks):

        pos_tracks_grouped = tracks

        #hash?? there seems like there could be some quick hashing
        #to simplify this

        #big picture: take large track data and simplify greatly while also
        #leaving all valuable information and order in tact. Hmm.

        #PANDAS?? may introduce some latency but does have incredible grouping libraries

        #correlated = {}
        #ms = MeanShift()
        #ms.fit(tracks)
        #labels = ms.labels_
        #cluster_centers = ms.cluster_centers_
        ## for i in
        #return

        return pos_tracks_grouped

    # returns a sorted list of probabilities of every other point being part of the same targets as the starting point
    def distanceTo(self, point, tracks):
        return


    # This finds new targets in the data
    def findNewTargets(self, usable_tracks):
        return

    ## heres the clear markers function that we use two different versions abov --cole
    def clearMarkers(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.type = 1  # Cube
        marker.action = 3  # delete all
        self.marker_array.markers.clear()
        self.marker_array.markers.append(marker)
        self.viz_targets.publish(self.marker_array)



    # for other targeting system
    # def meanShiftCenters(self, tracks):
    #     correlated = {}
    #     ms = MeanShift()
    #     ms.fit(tracks)
    #     labels = ms.labels_
    #     cluster_centers = ms.cluster_centers_
    #     # for i in
    #     return
