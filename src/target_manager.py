from target import *
import statistics
from visualization_msgs.msg import MarkerArray

##COLE - FOR DB SCAN REMOVE IF GO ANOTHER DIRECTION
from sklearn.cluster import DBSCAN

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


        #eps = cluster distance, min_samples is how many points to consider a cluster
        #BUT-- is this efficient?
        # ****** n_jobs param allows us to THREAD!!! POSSIBLE BIG SPEED-UP *******


        cluster = DBSCAN(eps=3, min_samples=6).fit(tracks)

        # clus_test = len(set(labels)) - (1 if -1 in labels else 0)
        # nois_test = list(labels).count(-1)

        # print('Estimated num of clusters: %d' % clus_test)
        # print('Estimated num of noise points: %d' % nois_test)


        # feels like I'm not returning the right thing, I might actually be looking for a subset
        # of the following data? Investigate.
        return cluster
    


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
