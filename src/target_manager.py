from target import *
from math import *
import statistics
from visualization_msgs.msg import MarkerArray
import time

# This is for grouping
# import numpy as np
# from sklearn.cluster import MeanShift

# THIS CLASS MANAGES ALL OF THE TARGETS


class TargetManager:
    def __init__(self):
        # super().__init__()
        # a list of active targets
        self.active_targets = []
        self.viz_targets = rospy.Publisher('targets_array_viz', MarkerArray, queue_size=100)
        self.marker_array = MarkerArray()
        self.start = time.time()

    # take in raw tracks and add to existing target or create new target
    # more of a callback
    def update(self, tracks):

        stragglers = self.grouper(tracks)
        # probability of each centroid going to each target
        # check for conflicts
        # print("stragglers:", len(stragglers))
        # Go through what remians and find more targets until random noise is left
        left_overs = self.findNewTargets(stragglers)

        self.clearMarkers()

        # clear marker array
        self.marker_array.markers.clear()

        # turn active targts into markers
        for target in self.active_targets:
            # add this new marker to the array 
            self.marker_array.markers.append(target.marker)
        
        # print(len(self.marker_array.markers))
        # publish the marker array
        self.viz_targets.publish(self.marker_array)
        # print("updated")



    # This creates the selection criteria for divvying up the tracks for tracks
    def selection_criteria(self, sorted_tracks):
        #edge check for garbage
        if sorted_tracks[0][1] < 1000 or len(sorted_tracks) <= 9:
            return  False

        # Sorted track has the structure (track, probability)
        probabilities = [x[1] for x in sorted_tracks]
        std_dev_mult = statistics.stdev(probabilities)


        new_returnable_tracks = []

        for track in sorted_tracks:
            if track[1] >= std_dev_mult:
                new_returnable_tracks.append(track[0])

        return new_returnable_tracks


    # This looks for groups of targets
    # USE DISTANCE AND VELOCITY TO GROUP
    def grouper(self, tracks):
        dt = time.time() - self.start
        usable_tracks = tracks
        # if there are no active targets make new ones based on data
        if(len(self.active_targets) != 0):
            for target in self.active_targets:
                probabilities = self.distanceTo(
                    target.getPred(), usable_tracks)
                # something = 2 # YOU NEED TO DETERMINE THE METRIC BASED ON THE DATA
                # candidate_tracks = [p for p in proabilities if p.prob >= something]
                candidate_tracks = self.selection_criteria(probabilities)
                if candidate_tracks != False:
                    # YOU NEED TO FIGURE OUT SOMETHING BETTER FOR THE dt THERE
                    target.updateTarget(candidate_tracks, dt)
                    # Remove tracks used for this target
                    usable_tracks = list(set(usable_tracks) ^
                                        set(candidate_tracks))
                else:
                    self.active_targets.remove(target)
        self.start = time.time()
        return usable_tracks



    # returns a sorted list of probabilities of every other point being part of the same targets as the starting point
    def distanceTo(self, point, tracks):
        distances = []
        for track in tracks:
            new_del = Delta(point, track).prob
            distances.append((track, new_del))


        # sorting
        sorted_distances = sorted(
            distances, key=lambda relation: relation[1], reverse=True)
        # print(sorted_distances)
        return sorted_distances


    # This finds new targets in the data
    def findNewTargets(self, usable_tracks):
        for track in usable_tracks:
            # print("new target usable:", len(usable_tracks))
            probabilities = self.distanceTo(track, usable_tracks)
            # something = 2 # YOU NEED TO DETERMINE THE METRIC BASED ON THE DATA
            # probable_tracks = [p for p in probabilities if p.prob >= something]
            candidate_tracks = self.selection_criteria(probabilities)
            if candidate_tracks != False:    
                new_target = Target(candidate_tracks)
                # Remove the tracks for the new target from the list
                usable_tracks = list(set(usable_tracks) ^ set(candidate_tracks))
                self.active_targets.append(new_target)
        return usable_tracks


    # for other targeting system
    # def meanShiftCenters(self, tracks):
    #     correlated = {}
    #     ms = MeanShift()
    #     ms.fit(tracks)
    #     labels = ms.labels_
    #     cluster_centers = ms.cluster_centers_
    #     # for i in
    #     return

    def clearMarkers(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.type = 1  # Cube
        marker.action = 3  # delete all
        self.marker_array.markers.clear()
        self.marker_array.markers.append(marker)
        self.viz_targets.publish(self.marker_array)


class Delta:
    def __init__(self, track1, track2):
        self.track1 = track1
        self.track2 = track2
        self.dx = abs(self.track1.cart_x - self.track2.cart_x)
        self.dy = abs(self.track1.cart_y - self.track2.cart_y)
        self.dvx = abs(self.track1.rate_x - self.track2.rate_x)
        self.dvy = abs(self.track1.rate_y - self.track2.rate_y)
        self.dist = math.sqrt(self.dx**2 + self.dy**2)
        self.vel_del = math.sqrt(self.dvx**2 + self.dvy**2)
        self.prob = self.probability()


    # This calculates the probability of the two points being part of the same target - WORKING
    def probability(self):
        # EDITED BY COLE (WAS ORIG INT VALUES, CHANGED MULTIPLIERS)
        # Changed nums
        # less of a distace delta means a higher probability
        p_dis = 1/((self.dist + 0.00001)**2)
        # lower velocity delta means a higher probability
        p_vel = 1/((self.vel_del + 0.00001)**2)
        # This changes how much of the metric is based on distance
        dist_mult = 0.8
        # This changes how much of the metric is based on velocity
        vel_mult = 3.0

        return (p_dis * dist_mult) * (p_vel * vel_mult)
