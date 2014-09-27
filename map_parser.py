import math 

class map_obj(object):
    def __init__(self, map_fn):
        x = 0
        self.valid_coordinates = []
        with open(map_fn) as fin:
          line = fin.next()
          line = line.rstrip().split()
          if (line[0] == 'robot_specifications->global_mapsize_x'):
            pass
          elif (line[0] == 'robot_specifications->global_mapsize_y'):
            pass
          elif (line[0] == 'robot_specifications->resolution'):
            self.resolution = 10
          elif (line[0] == 'robot_specifications->autoshifted_x'):
            pass
          elif (line[0] == 'robot_specifications->autoshifted_y'):
            pass
          elif (line[0] == 'global_map[0]:'):
            self.mapsize_x = int(line[1])
            self.mapsize_y = int(line[2])
            self.grid = np.zeros((self.mapsize_x, self.mapsize_y), dtype=np.float64)

          else:
            # read 800 occupancy cells. 
            # update self.grid
            for y in range(self.mapsize_y):
              self.grid[x, y] = np.float64(line[y])
              if (self.grid[x,y] < 1 and self.grid[x,y] >=0):
                self.valid_coordinates.append((x, y))
            x += 1


        #iterate over positions in the map,
        #if it's valid add to valid coordinates
        # self.valid_coordinates.append(xy(x, y))

    def get_valid_coordinates(self):
      return self.valid_coordinates

    def is_hit(self, x,y):
      p = self.grid[ int(x / self.resolution), int(y / self.resolution) ] 
      return p > 0.8 or p < 0

    def ray_finding(self, pose):
      x, y = pose[0], pose[1] 
      dx, dy = math.cos(pose[2]), math.sin(pose[1])
      dist = 0
      is_hit = self.is_hit(x,y)
      while !is_hit:
        x += dx
        y += dy
        dist += 1.0
        is_hit = self.is_hit(x,y) 
    
      return dist
