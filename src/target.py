# This is a class for objectifying targets
import rospy
from visualization_msgs.msg import Marker
import time
import numpy as np
import random

Q_CONST = 2.0

# Pose object


class Pose:
    def __init__(self, x, y, rate_x, rate_y):
        # super().__init__()
        self.x = x
        self.y = y
        self.rate_x = rate_x
        self.rate_y = rate_y

        # this is for compatability wih Delta in target_manager.py
        self.cart_x = x
        self.cart_y = y


# every iteration, predict then update
# update takes a 'measurement' aka what is the predicted centroid which is np.array([x,y,vx,vy])
class kf:
    def __init__(self, x, P, F, H, R, Q):
        # LOOK AT ilectureonlin kf lecture 7
        # state matrix [x, y, velx, vely]
        self.x = x
        # state cov mtx
        self.P = P
        # transition mtx
        self.F = F
        # observation model
        self.H = H
        # sensor noise cov
        self.R = R
        # process noise cov
        self.Q = Q

    def predict(self, *args, **kwargs):
        if len(args) > 0:
            t = self.F @ self.x
            return t
        else:
            self.x = self.F @ self.x
            self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        # print("printing h: ", self.H)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        y = z - self.H @ self.x
        self.x += K @ y
        self.P = self.P - K @ self.H @ self.P

    def update_dt(self, dt):
        self.F = np.array([[1, 0, dt, 0.0],
                           [0, 1, 0, dt],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])

    # returns [x,y] in a array
    def get_current_state(self):
        return self.x


class Target:
    def __init__(self, init_tracks):
        # super().__init__()
        # store all of the tracks used for this object
        self.tracks_history = []
        # this is all of the poses that the target has had
        self.pose_history = []

        self.current_pose = None
        # These are the tracks being used at this time
        self.working_tracks = []

        self.initTarget(init_tracks)

        self.marker = Marker()
        self.marker.header.frame_id = "RadarFrontCenter"
        self.marker.type = 1  # Cube
        self.marker.action = 0  # Add
        # eventually this should be determined by the extimated size of the physical target
        self.marker.scale.x = 1.0
        self.marker.scale.y = 1.0
        self.marker.scale.z = 1.0
        self.marker.color.r = 1.0
        self.marker.color.b = 0.0
        self.marker.color.g = 0.0
        self.marker.color.a = 1.0
        self.marker.pose.orientation.w = 1.0
        self.marker.pose.position.x = self.current_pose.x
        self.marker.pose.position.y = self.current_pose.y
        # radar is 2d so this can be left at 0.0
        self.marker.pose.position.z = 0.0
        self.marker.id = random.randint(0, 1000000)

        self.start = time.time()

        # delta t for kf initialy
        dt = 0.1

        self.kf = kf(x=np.array([self.current_pose.x, self.current_pose.y, self.current_pose.rate_x, self.current_pose.rate_y]),
                     P=np.diag([0, 0, 0, 0.0]),
                     F=np.array([[1, 0, dt, 0.0],
                                 [0, 1, 0, dt],
                                 [0, 0, 1, 0],
                                 [0, 0, 0, 1]]),
                     H=np.array([[1, 0, 0, 0.0],
                                 [0, 1, 0, 0],
                                 [0, 0, 1, 0],
                                 [0, 0, 0, 1]]),
                     R=np.diag([10.0, 10.0, 10.0, 10.0]),
                     Q=np.array([[0.028, 0.0, 0.28, 0.0],
                                 [0.0, 0.28, 0.0, 0.28],
                                 [0.28, 0.0, 2.8, 0.0],
                                 [0.0, 0.28, 0.0, 2.8]]) * Q_CONST)

    # This gets the target started
    def initTarget(self, tracks):
        # print("new target")
        x, y, vx, vy = self.avgTracks(tracks)
        self.tracks_history.append(tracks)
        pose = Pose(x, y, vx, vy)
        self.updatePose(pose)


    # This takes an average of each component of a track over all the candidate tracks
    def avgTracks(self, candidates):
        x = 0
        y = 0
        rate_x = 0
        rate_y = 0
        t = 0
        for track in candidates:
            x += track.cart_x
            y += track.cart_y
            rate_x += track.rate_x
            rate_y += track.rate_y
            t += 1
        return x/t, y/t, rate_x/t, rate_y/t

    # Part of the kf that does prediction

    def getPred(self):
        state = self.kf.predict(True)
        p = Pose(*state)
        return p

    # THIS UPDATES THE TARGET WITH NEW TRACKS

    def updateTarget(self, new_tracks, dt):
        self.tracks_history.append(self.working_tracks)
        self.working_tracks = new_tracks
        # THESE VALUES
        x, y, vx, vy = self.avgTracks(new_tracks)

        dt = time.time() - self.start
        self.kf.update_dt(dt)

        data = np.array([x, y, vx, vy])
        self.kf.predict()
        self.kf.update(data)

        filtered = self.kf.get_current_state()
        pos = Pose(filtered[0], filtered[1], filtered[2], filtered[3])
        self.updatePose(pos)
        self.start = time.time()

    def updatePose(self, pose):
        self.current_pose = pose
        self.pose_history.append(self.current_pose)

    def estimateSize(self):
        # use the working tracks to 
        return


class Thing:
    def __init__(self, init_tracks):
        # super().__init__()
        # store all of the tracks used for this object
        self.tracks_history = []
        # this is all of the poses that the target has had
        self.pose_history = []

        self.current_pose = None
        # These are the tracks being used at this time
        self.working_tracks = []

        self.initTarget(init_tracks)

        self.marker = Marker()
        self.marker.header.frame_id = "RadarFrontCenter"
        self.marker.type = 1  # Cube
        self.marker.action = 0  # Add
        # eventually this should be determined by the extimated size of the physical target
        self.marker.scale.x = 1.0
        self.marker.scale.y = 1.0
        self.marker.scale.z = 1.0
        self.marker.color.r = 1.0
        self.marker.color.b = 0.0
        self.marker.color.g = 0.0
        self.marker.color.a = 1.0
        self.marker.pose.orientation.w = 1.0
        self.marker.pose.position.x = self.current_pose.x
        self.marker.pose.position.y = self.current_pose.y
        # radar is 2d so this can be left at 0.0
        self.marker.pose.position.z = 0.0
        self.marker.id = random.randint(0, 1000000)

        self.start = time.time()

        # delta t for kf initialy
        dt = 0.1

        self.kf = kf(x=np.array([self.current_pose.x, self.current_pose.y, self.current_pose.rate_x, self.current_pose.rate_y]),
                     P=np.diag([0, 0, 0, 0.0]),
                     F=np.array([[1, 0, dt, 0.0],
                                 [0, 1, 0, dt],
                                 [0, 0, 1, 0],
                                 [0, 0, 0, 1]]),
                     H=np.array([[1, 0, 0, 0.0],
                                 [0, 1, 0, 0],
                                 [0, 0, 1, 0],
                                 [0, 0, 0, 1]]),
                     R=np.diag([10.0, 10.0, 10.0, 10.0]),
                     Q=np.array([[0.028, 0.0, 0.28, 0.0],
                                 [0.0, 0.28, 0.0, 0.28],
                                 [0.28, 0.0, 2.8, 0.0],
                                 [0.0, 0.28, 0.0, 2.8]]) * Q_CONST)
    # This gets the target started
    def initTarget(self, tracks):
        # print("new target")
        x, y, vx, vy = self.avgTracks(tracks)
        self.tracks_history.append(tracks)
        pose = Pose(x, y, vx, vy)
        self.updatePose(pose)


    # This takes an average of each component of a track over all the candidate tracks
    def centroid(self, candidates):
        x = 0
        y = 0
        rate_x = 0
        rate_y = 0
        t = 0
        for track in candidates:
            x += track.cart_x
            y += track.cart_y
            rate_x += track.rate_x
            rate_y += track.rate_y
            t += 1
        return x/t, y/t, rate_x/t, rate_y/t

    # Part of the kf that does prediction

    def getPred(self):
        state = self.kf.predict(True)
        p = Pose(*state)
        return p

    # THIS UPDATES THE TARGET WITH NEW TRACKS

    def updateTarget(self, new_tracks, dt):
        self.tracks_history.append(self.working_tracks)
        self.working_tracks = new_tracks
        # THESE VALUES
        x, y, vx, vy = self.avgTracks(new_tracks)

        dt = time.time() - self.start
        self.kf.update_dt(dt)

        data = np.array([x, y, vx, vy])
        self.kf.predict()
        self.kf.update(data)

        filtered = self.kf.get_current_state()
        pos = Pose(filtered[0], filtered[1], filtered[2], filtered[3])
        self.updatePose(pos)
        self.start = time.time()

    def updatePose(self, pose):
        self.current_pose = pose
        self.pose_history.append(self.current_pose)

    def viz(self):
        return