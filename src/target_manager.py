from target import *
from math import *

# THIS CLASS MANAGES ALL OF THE TARGETS 
class TargetManager:
    def __init__(self):
        super().__init__()
        # a list of active targets
        self.active_targets = []

    # take in raw tracks and add to existing target or create new target
    # more of a callback
    def update(self, tracks):
        stragglers = self.grouper(tracks)
        # probability of each centroid going to each target
            # check for conflicts


    # This creates the selection criteria for divvying up the tracks for tracks
    def selection_criteria(self, sorted_tracks):
        # uhhh get some top percentage of tracks
        # This needs work and should be something smarter
        # get the best points
        # look at the data maybe?
        ten_percent = len(sorted_tracks)//10
        return sorted_tracks[:ten_percent]


    # This looks for groups of targets
    # USE DISTANCE AND VELOCITY TO GROUP
    def grouper(self, tracks):
        usable_tracks = tracks
        # if there are no active targets make new ones based on data
        if(len(self.active_targets) == 0):
            for track in usable_tracks:
                probabilities = self.distanceTo(track, usable_tracks)
                something = 2 # YOU NEED TO DETERMINE THE METRIC BASED ON THE DATA
                probable_tracks = [p for p in probabilities if p.prob >= something]
                new_target = Target(probable_tracks)
                # Remove the tracks for the new target from the list
                usable_tracks = list(set(usable_tracks)^set(candidate_tracks))
                self.active_targets.append(new_target)
        # Use active targets for divvying up tracks
        else:
            for target in self.active_targets:
                probabilities = self.distanceTo(target.current_pose, usable_tracks)
                something = 2 # YOU NEED TO DETERMINE THE METRIC BASED ON THE DATA
                candidate_tracks = [p for p in proabilities if p.prob >= something]
                target.update(candidate_tracks)
                # Remove tracks used for this target
                usable_tracks = list(set(usable_tracks)^set(candidate_tracks))
            if(len(usable_tracks) != 0):
                # do something with the unused tracks
                # identify new targets and add to list

        # These are the stragglers
        return usable_tracks


    def distanceTo(self, point, tracks):
        distances = {}
        for track in tracks:
            new_del = Delta(point, track)
            distances[track] = new_del

        sort_distances = sorted(distances.items(), key=lambda x: x.prob, reverse=True)
        print(sort_distances)
        return sorted_distances



class Delta:
    def __init__(self, track1, track2):
        super().__init__()
        self.track1 = track1
        self.track2 = track2
        self.dx = abs(self.track1.cart_x - self.track2.cart_x)
        self.dy = abs(self.track1.cart_y - self.track2.cart_y)
        self.dvx = abs(self.track1.rate_x - self.track2.rate_x)
        self.dvy = abs(self.track1.rate_y - self.track2.rate_y)
        self.dist = sqrt(self.dx^2 + self.dy^2)
        self.vel_del = sqrt(self.dvx^2 + self.dvy^2)
        self.prob = self.probability()
    
    # This calculates the probability of the two points being part of the same target
    def probability(self):
        # less of a distace delta means a higher probability
        p_dis = 1/(self.dist^1.0)
        # lower velocity delta means a higher probability 
        p_vel = 1/(self.vel_del^1.0)
        # This changes how much of the metric is based on distance
        dist_mult = 1
        # This changes how much of the metric is based on velocity
        vel_mult = 1

        return (p_dis * dist_mult) * (p_vel * vel_mult)
