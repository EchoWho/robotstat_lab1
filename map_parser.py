
class xy(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y

class pose(object):
    def __init__(self, xy, theta):
        self.xy = xy
        self.theta = theta

class map_obj(object):
    def __init__(self, map_fn):
        self.valid_coordinates = []

        #iterate over positions in the map,
        #if it's valid add to valid coordinates
        # self.valid_coordinates.append(xy(x, y))

 
