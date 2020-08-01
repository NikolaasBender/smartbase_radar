# This is a class for objectifying targets
import rospy

# Pose object
class Pose:
    def __init__(self, x, y, rate_x, rate_y):
        super().__init__()
        self.x = x
        self.y = y
        self.rate_x = rate_x
        self.rate_y = rate_y


# every iteration, predict then update
# update takes a 'measurement' aka what is the predicted centroid which is np.array([x,y])
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

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
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
        super().__init__()
        # store all of the tracks used for this object
        self.tracks_history = {}
        # this is all of the poses that the target has had
        self.pose_history = []

        self.current_pose
        # These are the tracks being used at this time
        self.working_tracks = {}

        # delta t for kf
        self.dt = 0.1


        self.kf(x=np.array([self.current_pose.x, self.current_pose.y, self.current_pose.rate_x, self.current_pose.rate_y]),
              
              P=np.diag([0, 0, 0, 0.0]),
              F=np.array([[1, 0, dt, 0.0],
                          [0, 1, 0, dt],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]]),
              H=np.array([[1, 0, 0, 0.0],
                          [0, 1, 0, 0]]),
              R=np.diag([10.0, 10.0]),
              Q=np.array([[0.028, 0.0, 0.28, 0.0],
                          [0.0, 0.28, 0.0, 0.28],
                          [0.28, 0.0, 2.8, 0.0],
                          [0.0, 0.28, 0.0, 2.8]]) * Q_CONST)



    # fuses raw tracks into a target
    def fuse_tracks(self, all_tracks):
        # get kf prediction for where this track is moving
        self.get_pred()

        probable_tracks = []

        # find the tracks closest to this target
        for track in all_tracks:
            if self.is_close():
                probable_tracks.append(track)
        
        # the average candidate components
        c_x, c_y, c_rx, c_ry = self.avg_tracks(probable_tracks)


    # This takes an average of each component of a track over all the candidate tracks
    def avg_tracks(self, candidates):
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
    def get_pred(self):
        return


    # This decides wether or not a track is close enough to the target to be considered
    def is_close(self):
        return


