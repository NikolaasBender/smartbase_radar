import rospy
import math

class Track:
    def __init__(self, track_range, track_azimuth, track_rate, track_rcs, radar_placement):
        super().__init__()
        self.t_range = track_range
        self.t_azimuth = track_azimuth
        self.t_rate = track_rate
        self.t_rcs = track_rcs
        self.radar_place = radar_placement
        self.id = self.gen_id()

        # the cartesian coordinates of the track
        self.cart_x
        self.cart_y

        self.rate_x
        self.rate_y

    # convert the range and azimuth to a cartesian position
    def to_cartesian(self):
        # t_range may actually be affected by radar_placement. investigating.
        self.cart_x = self.t_range * math.sin(self.t_azimuth)
        self.cart_y = self.t_range * math.sin(self.t_azimuth)
