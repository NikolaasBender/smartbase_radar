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
        self.tracks_history = []
        # this is all of the poses that the target has had
        self.pose_history = []

        self.current_pose = None
        # These are the tracks being used at this time
        self.working_tracks = []

        self.initTarget(init_tracks)

        # delta t for kf initialy
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

    # This gets the target started
    def initTarget(self, tracks):
        x, y, vx, vy = self.avgTracks(tracks)
        self.tracks_history.append(tracks)
        pose = Pose(x, y, vx, vy)
        self.updatePose(pose)
        # COLE YOU MAY NEED TO KF STUFF IN HERE TOO


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
    def get_pred(self):
        # COLE I NEED THIS TO SPIT OUT x, y, vx, vy FOR USE ELSEWHERE
        return


    # THIS UPDATES THE TARGET WITH NEW TRACKS
    def updateTarget(self, new_tracks, dt):
        self.tracks_history.append(self.working_tracks)
        self.working_tracks = new_tracks
        self.dt = dt

        # THESE VALUES
        x, y, vx, vy = self.avg_tracks(new_tracks)
        # COLE YOU NEED TO USE THE ABOVE VALUES FOR FUSION WITH THE KF

        # COLE YOU NEED TO UPDATE THE self.current_pose WITH WHAT YOU GET FROM THE KF
        self.updatePose(coles_pose) # CHANGE THE VARIABLE NAME, I'M JUST MAKING A POINT NOW

    def updatePose(self, pose):
        self.current_pose = pose
        self.pose_history.append(self.current_pose)
        


# STILL DECIDING WHERE THIS NEEDS TO GO
#     def calcDists(self, point, tracks):
#         distances = {}
#         for track in tracks:
#             new_del = Delta(point, track)
#             distances[track] = new_del

#         sort_distances = sorted(distances.items(), key=lambda x: x.prob, reverse=True)
#         print(sort_distances)
#         return sorted_distances


# class Delta:
#     def __init__(self, track1, track2):
#         super().__init__()
#         self.track1 = track1
#         self.track2 = track2
#         self.dx = abs(self.track1.cart_x - self.track2.cart_x)
#         self.dy = abs(self.track1.cart_y - self.track2.cart_y)
#         self.dvx = abs(self.track1.rate_x - self.track2.rate_x)
#         self.dvy = abs(self.track1.rate_y - self.track2.rate_y)
#         self.dist = sqrt(self.dx^2 + self.dy^2)
#         self.vel_del = sqrt(self.dvx^2 + self.dvy^2)
#         self.prob = self.probability()

#     # This calculates the probability of the two points being part of the same target
#     def probability(self):
#         # less of a distace delta means a higher probability
#         p_dis = 1/(self.dist^1.0)
#         # lower velocity delta means a higher probability
#         p_vel = 1/(self.vel_del^1.0)
#         # This changes how much of the metric is based on distance
#         dist_mult = 1
#         # This changes how much of the metric is based on velocity
#         vel_mult = 1

#         return (p_dis * dist_mult) * (p_vel * vel_mult)
