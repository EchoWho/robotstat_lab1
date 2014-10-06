import os, sys, pdb

import gtkutils.img_util as iu
import map_parser
import logparse
import motion_model
import obssensemodels
import pdbwrap as pdbw
import numpy
import math
import numpy as np
import copy

import matplotlib.pyplot as plt
import matplotlib
from numpy.random import multivariate_normal

from multiprocessing import Pool

def isobservation(line):
    return line[0] == 'L'

def ismotion(line):
    return line[0] == 'O'

class particle(object):
    def __init__(self, pose, weight, gamma = 0):
        self.pose = pose
        self.weight = weight
        self.gamma = gamma 

class particle_collection(object):
    def __init__(self, n_particles, map_obj, nbr_theta = 10, nbr_gamma=36):
        self.map_obj = map_obj
        self.n_particles = n_particles
        self.particles = []
        self.max_ratio = 100
        self.last_scatter = None

        unit_theta = 2 * math.pi / float(nbr_theta)
        unit_gamma = 2 * math.pi / float(nbr_gamma)
        vec_pos = map_obj.get_valid_coordinates()
        num_pos = len(vec_pos)
        
        self.fig = plt.figure(num = 1, figsize = (15, 15))
        


        map_orient = numpy.rot90(numpy.fliplr(self.map_obj.hit_map.copy()))

        imgplot = plt.imshow(map_orient)
        plt.show(block = False)

        for p_idx in range(n_particles):
          pos_idx = int(numpy.random.uniform(0, num_pos - 1e-6))
          pos = vec_pos[pos_idx]
          theta_i = int(numpy.random.uniform(0, nbr_theta - 1e-6))
          gamma_i = int(numpy.random.uniform(0, nbr_gamma - 1e-6))

          self.particles.append(particle(numpy.array([self.map_obj.resolution * pos[0], 
                                                      self.map_obj.resolution * pos[1], 
                                                      theta_i * unit_theta]),
                                         1.0, 
                                         gamma_i * unit_gamma))


        # numpy.random.shuffle(vec_pos)
        # random_positions = vec_pos[:n_particles]
        # random_positions = numpy.random.choice(vec_pos, size = n_particles,
        #                                        replace = False)

        
        #for pos in random_positions:
        #    for theta_i in range(nbr_theta):
        #      for gamma_i in range(nbr_gamma):
        #        self.particles.append(particle(numpy.array([self.map_obj.resolution * pos[0], 
        #                                                    self.map_obj.resolution * pos[1], 
        #                                                    theta_i * unit_theta]),
        #                                       1.0, 
        #                                       gamma_i * unit_gamma))

        print "created {} particles".format(len(self.particles))

    def show(self):
        if self.last_scatter is not None:
            self.last_scatter.remove()

        hm = self.map_obj.hit_map.copy()
        canvas = 255 * numpy.dstack((numpy.zeros_like(hm), hm, hm)).astype('uint8')

        color = [255, 128, 0]
        
        pose_coords = numpy.asarray([self.map_obj.get_pose_coord(p.pose) for p in self.particles])
        x = pose_coords[:, 0]
        y = pose_coords[:, 1]
        
        
        self.last_scatter = plt.scatter(x, y, c='c')
        plt.axis([0, 800, 800, 0])
        plt.show(block = False)
        self.fig.canvas.draw()
        
        # for p in self.particles:
        #     pose_coord = self.map_obj.get_pose_coord(p.pose)

        #     canvas[pose_coord[0] - 1, pose_coord[1] - 1, :] = color
        #     canvas[pose_coord[0] - 1, pose_coord[1], :] = color
        #     canvas[pose_coord[0] - 1, pose_coord[1] + 1, :] = color

        #     canvas[pose_coord[0] + 1, pose_coord[1] + 1, :] = color
        #     canvas[pose_coord[0] + 1, pose_coord[1], :] = color
        #     canvas[pose_coord[0] + 1, pose_coord[1] - 1, :] = color

        #     canvas[pose_coord[0], pose_coord[1] - 1, :] = color
        #     canvas[pose_coord[0], pose_coord[1] + 1, :] = color

        #     canvas[pose_coord[0], pose_coord[1], :] = color
            
        # iu.v(canvas)

    def get_weights(self):
        return numpy.array([p.weight for p in self.particles])

    def resample(self):
        #shuffle? shuffle.
        numpy.random.shuffle(self.particles)

        weights = self.get_weights()
        max_weight = weights.max()
        min_weight = max_weight / self.max_ratio
        weights[np.array(map(lambda x: 0 < x < min_weight, weights))] = min_weight
        assert(weights.max() > 0)

        w_cumsums = np.cumsum(weights)

        #do it
        M = len(weights)
        inc = w_cumsums[-1] / np.float64(M)
        w = 0
        idx = 0
        selected = []


        gamma_cov = numpy.pi / 180 * 2.
        for i in range(M):
          selected.append(copy.deepcopy(self.particles[idx]))
          selected[-1].weight = 1        
          selected[-1].gamma = numpy.random.normal(selected[-1].gamma, gamma_cov)
          w += inc
          while idx < len(w_cumsums) and w >= w_cumsums[idx]:
            idx += 1


        self.particles = selected
        
#### HACK ALERT FIXME the function run in parallel must be in global context ##
def obs_update(args):
  p = args[0]
  obs_model = args[1]
  laser_pose_offset = args[2]
  laser = args[3]
  p.weight *= obs_model.get_weight(p.pose, laser_pose_offset, laser)
  return p

def main():

    thread_pool = Pool(16)

    map_file = 'data/map/wean.dat'

    mo = map_parser.map_obj(map_file)
    
    logfile_fn = 'data/log/robotdata1.log'
    log = logparse.logparse(logfile_fn)
    
    n_particles = 1000
    print "creating particle collection of {} particles".format(n_particles)
    pc = particle_collection(n_particles = n_particles,
                             map_obj = mo,
                             nbr_theta = 10,
                             nbr_gamma = 50)
    print "created particle collection"

    num_new_motions = 0
    num_new_observations = 0
    
    odom_control_gen = motion_model.odometry_control_generator()
    obs_model = obssensemodels.observation_model(map_obj = mo)
    mm = motion_model.motion_model()

    # mo.show()
    print "showing pc"
    pc.show()


    # pose = pc.particles[200].pose
    # mo.vis_z_expected(pose)
    # obs_model.vis_p_z_given_x_u(pose)

    for (l_idx, line) in enumerate(log.lines):
        line = line.split()

        print "line {} / {}".format(l_idx + 1, len(log.lines))

        if ismotion(line):
            num_new_motions += 1
            pose = numpy.array([np.float64(line[1]), np.float64(line[2]), np.float64(line[3])])
            u = odom_control_gen.calculate_u(pose)
            for p in pc.particles: 
                mm.update(p, u)
            
            print "motion : {}".format(u)

        elif isobservation(line):
            num_new_observations += 1
            # 0. list of 180 laser readings [ float ] * 180 
            # 1. for each particle compute P(Z | X) = \prod P(Z_i | X)  i =0... 179
            #    1.1. E[Z_i | X] = func1(map_obj, cur_pose) 
            #    1.2 P(Z_i |X) = func2( E[Z_i] )
            #    1.3 weight = P(Z |X) = \prod 180 ...

            #  combine 1.1. and 1.2 as P(Z |X) = func(map_obj, cur_pose)
            laser_pose_offset = (np.float64(line[4]) - np.float64(line[1]), 
                                 np.float64(line[5]) - np.float64(line[2]), 
                                 np.float64(line[6]) - np.float64(line[3]))
            laser = [ ]
            laser_start = 7
            n_in_wall = 0
            for i in range(180):
                laser.append(np.float64(line[i + laser_start]))

#            n_particles = len(pc.particles)
#            obs_update_args = zip(pc.particles, 
#                                  [obs_model] * n_particles, 
#                                  [laser_pose_offset] * n_particles,
#                                  [laser] * n_particles) 
#            pc.particles = thread_pool.map(obs_update, obs_update_args) 

# IF not parallelizing
            for p_idx, p in enumerate(pc.particles):
              p.weight *= obs_model.get_weight(p.pose, laser_pose_offset, laser)

            new_weights = pc.get_weights()
            print "max weight: {}".format(new_weights.max())
            print "max weight location: {}".format( pc.particles[np.argmax(new_weights)].pose )

        else:
            raise RuntimeError("unknown line type!!!11!!!1")

        if (num_new_motions > 0) and (num_new_observations > 0):
            num_new_motions = 0
            num_new_observations = 0
            pc.resample()
            #update stuff

        if l_idx % 5 == 0:
            pc.show()
        #   mo.vis_particles(pc.particles)

    print "DONE"

if __name__ == '__main__':
    numpy.random.seed(seed=7111990)
    pdbw.pdbwrap(main)()
