import rospy
import math


# # This is for the radar messages defined in this pkg
# class Track:
#     def __init__(self, track):
#         # super().__init__()
#         self.t_range = track.range
#         self.t_azimuth = track.azimuth
#         self.t_rate = track.range_rate
#         self.t_rcs = track.rcs
#         # self.radar_place = radar_placement
#         self.status = track.status

#         # the cartesian coordinates of the track
#         self.cart_x = self.t_range * math.cos(self.t_azimuth)
#         self.cart_y = self.t_range * math.sin(self.t_azimuth)

#         self.rate_x = self.t_rate * math.cos(self.t_azimuth)
#         self.rate_y = self.t_rate * math.sin(self.t_azimuth)

#         self.id = self.gen_id()

#     # This generates a hash of this radar track for later
#     def gen_id(self):
#         return hash((self.cart_x, self.cart_y, self.rate_x, self.rate_y, self.t_azimuth, self.t_range, self.t_rate))

# This is for the radar messages defined in radar driver
class Track:
    def __init__(self, track):
        # super().__init__()
        self.t_range = track.range
        # In radians
        self.t_azimuth = track.angle
        self.t_accel = track.accel
        self.t_rate = track.rate
        # Lateral velocity relative to radar
        self.t_late_rate = track.late_rate
        # self.radar_place = radar_placement
        self.status = track.status
        self.abs_rate = track.absolute_rate
        self.move = track.moving
        self.pow = track.power
        self.num = track.number

        # the cartesian coordinates of the track
        self.cart_x = self.t_range * math.sin(self.t_azimuth)
        self.cart_y = self.t_range * math.cos(self.t_azimuth)

        self.rate_x = self.t_rate * math.sin(self.t_azimuth)
        self.rate_y = self.t_rate * math.cos(self.t_azimuth)

        self.id = self.gen_id()

    # This generates a hash of this radar track for later
    def gen_id(self):
        return hash((self.cart_x, self.cart_y, self.rate_x, self.rate_y, self.t_azimuth, self.t_range, self.t_rate))