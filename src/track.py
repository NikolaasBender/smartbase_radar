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
        self.cart_x = self.t_range * math.cos(self.t_azimuth)
        self.cart_y = self.t_range * math.sin(self.t_azimuth)

        self.rate_x = self.t_rate * math.cos(self.t_azimuth)
        self.rate_y = self.t_rate * math.sin(self.t_azimuth)

    # This generates a hash of this radar track for later
    def gen_id(self):
        return hash((self.cart_x, self.cart_y, self.rate_x, self.rate_y, self.t_azimuth, self.t_range, self.t_rate))