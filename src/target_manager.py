from target import *

# THIS CLASS MANAGES ALL OF THE TARGETS 
class TargetManager:
    def __init__(self):
        super().__init__()
        # a list of active targets
        self.active_targets = []

    # take in raw tracks and add to existing target or create new target
    def update(self, tracks):
        groups = self.grouper(tracks)


    # This looks for groups of targets
    def grouper(self, tracks):

    