import os, sys

import map_parser
import logparse
import motion_model

import math

def isobservation(line):
    return line[0] == 'L'

def ismotion(line):
    return line[0] == 'O'

class particle(object):
    def __init__(self, pose, weight):
        self.pose = pose
        self.weight = weight

class particle_collection(object):
    def __init__(self, n_particles, map_obj, nbr_theta = 20):
        self.map_obj = map_obj
        self.n_particles = n_particles
        self.particles = []

        unit_theta = 2 * math.pi / nbr_theta
        vec_pos = map_obj.get_valid_coordinates()
        for theta_i in range(nbr_theta):
          for pos in enumerate(vec_pos):
              self.particles.append(particle((pos[0], pos[1], theta_i * unit_theta),1))

    def get_weights(self):
        return numpy.array([p.weight for p in self.particles])

    def resample(self):
        weights = self.get_weights()
        w_cumsums = np.cumsum(weights)

        #do it
        M = len(weights)
        inc = w_cumsums[-1] / np.float64(M)
        w = 0
        idx = 0
        selected = []
        for i in range(M):
          selected.append(self.particles[idx].copy())
          selected[-1].weight = 1
          w += inc
          while w >= w_cumsum[idx]:
            idx += 1

        self.particles = selected
        

def main():

    map_file = 'data/map/wean.dat'
    mo = map_parser.map_obj(map_file)

    logfile_fn = 'data/log/robotdata1.log'
    log = logparse.logparse(logfile_fn)
    
    
    n_particles = 100
    particle_collection = particle_collection(n_particles = n_particles,
                                              map_obj = mo)

    num_new_motions = 0
    num_new_observations = 0
    
    ocg = odometry_control_generator()
    obs_model = observation_model(map_obj)


    for line in log.lines:

        if ismotion(line):
            num_new_motions += 1
            pose = (np.float64(line[1]), np.float64(line[2]), np.float64(line[3]))
            u = ocg.calculate_u(pose)
            for p in particle_collection.particles: 
                motion_model.update(p, u)

        elif isobservation(line):
            num_new_observations += 1
            # 0. list of 180 laser readings [ float ] * 180 
            # 1. for each particlae compute P(Z | X) = \prod P(Z_i | X)  i =0... 179
            #    1.1. E[Z_i | X] = func1(map_obj, cur_pose) 
            #    1.2 P(Z_i |X) = func2( E[Z_i] )
            #    1.3 weight = P(Z |X) = \prod 180 ...

            #  combine 1.1. and 1.2 as P(Z |X) = func(map_obj, cur_pose)
            laser_pose_offset = (np.float64(line[4]) - np.float64(line[1]), 
                                 np.float64(line[5]) - np.float64(line[2]), 
                                 np.float64(line[6]) - np.float64(line[3]))
            laser = [ ]
            laser_start = 7
            for i in range(180):
              laser.append(np.float64(line[i + laser_start]))
            for p in particle_collection.particles:
              p.weight *= obs_model.get_weight(p.pose, laser_pose_offset, laser)
            
        else:
            raise RuntimeError("unknown line type!!!11!!!1")

        if (num_new_motions > 0) and (num_new_observations > 0):
            num_new_motions = 0
            num_new_observations = 0
            particle_collection.resample()
            #update stuff

if __name__ == '__main__':
    main()
