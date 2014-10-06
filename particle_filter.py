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
    def __init__(self, pose, weight):
        self.pose = pose
        self.weight = weight

class particle_collection(object):
    def __init__(self, n_particles, map_obj, nbr_theta = 10, fig_handle = None):
        self.map_obj = map_obj
        self.n_particles = n_particles
        self.particles = []
        self.max_ratio = 100
        self.last_scatter = None

        unit_theta = 2 * math.pi / float(nbr_theta)
        vec_pos = map_obj.get_valid_coordinates()
        num_pos = len(vec_pos)
        
        self.fig = fig_handle
        map_orient = self.map_obj.hit_map.copy().T

        plt.figure(1)
#        plt.subplot(211)
        imgplot = plt.imshow(map_orient)
        plt.show(block = False)

        # for i in range(200):
        #     delt= multivariate_normal(mean = np.array([0,0,0]), 
        #         cov = numpy.diag([50, 50, 10 / 180.0 * numpy.pi]))
        #     self.particles.append(particle(numpy.array([4000, 
        #                                                 4000, 
        #                                                 0]), 1.0))
        for p_idx in range(n_particles):
          pos_idx = int(numpy.random.uniform(0, num_pos - 1e-6))
          pos = vec_pos[pos_idx]
          theta_i = int(numpy.random.uniform(0, nbr_theta - 1e-6))

          self.particles.append(particle(numpy.array([self.map_obj.resolution * pos[0], 
                                                      self.map_obj.resolution * pos[1], 
                                                      theta_i * unit_theta]),
                                         1.0))


        # numpy.random.shuffle(vec_pos)
        # random_positions = vec_pos[:n_particles]
        # random_positions = numpy.random.choice(vec_pos, size = n_particles,
        #                                        replace = False)

        # print "created {} particles".format(len(self.particles))

    def show(self):
        if self.last_scatter is not None:
            self.last_scatter.remove()

        hm = self.map_obj.hit_map.copy()
        canvas = 255 * numpy.dstack((numpy.zeros_like(hm), hm, hm)).astype('uint8')

        color = [255, 128, 0]
        
        pose_coords = numpy.asarray([self.map_obj.get_pose_coord(p.pose) for p in self.particles])
        x = pose_coords[:, 0]
        y = pose_coords[:, 1]
        
        plt.figure(1,figsize = (20, 20))
#        plt.subplot(211)
        self.last_scatter = plt.scatter(x, y, c='c')
        plt.axis([0, 800, 0, 800])
        plt.show(block = False)
        plt.draw()
       
    def get_weights(self):
        return numpy.array([p.weight for p in self.particles])

    def resample(self):
        #shuffle? shuffle.
        numpy.random.shuffle(self.particles)

        weights = self.get_weights()
        # max_weight = weights.max()
        # min_weight = max_weight #/ self.max_ratio
        # weights[np.array(map(lambda x: 0 < x < min_weight, weights))] = min_weight
        assert(weights.max() > 0)

        w_cumsums = np.cumsum(weights)

        #do it
        M = len(weights)
        inc = w_cumsums[-1] / np.float64(M)
        w = 0
        idx = 0
        selected = []


        for i in range(M):
          selected.append(copy.deepcopy(self.particles[idx]))
          selected[-1].weight = 1        
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

    fig = plt.figure(num = 1, figsize = (20, 20))

#    sf1 = fig.add_subplot(2,1,1)
#    sf2 = fig.add_subplot(2,1,2)
    thread_pool = Pool(16)

    map_file = 'data/map/wean.dat'

    mo = map_parser.map_obj(map_file)
    
    logfile_fn = 'data/log/robotdata1.log'
    log = logparse.logparse(logfile_fn)
    
    n_particles = 200
    print "creating particle collection of {} particles".format(n_particles)
    pc = particle_collection(n_particles = n_particles,
                             map_obj = mo,
                             nbr_theta = 50,
                             fig_handle = fig)

    print "created particle collection"

    have_moved = True
    first_obs_at_pos = True
    num_new_motions = 0
    num_new_observations = 0
    
    odom_control_gen = motion_model.odometry_control_generator()
    obs_model = obssensemodels.observation_model(map_obj = mo)
    mm = motion_model.motion_model()
    obs_view = obssensemodels.observation_view(fig_handle = fig)

    # mo.show()
    #print "showing pc"
    #pc.show()

    #pose = pc.particles[200].pose
    # mo.vis_z_expected(pose)
    # obs_model.vis_p_z_given_x_u(pose)
    
    # TODO THERE IS A SKIP
    for (l_idx, line) in enumerate(log.lines[58:]):
        line = line.split()

        print "line {} / {}".format(l_idx + 1, len(log.lines))

        if ismotion(line) or isobservation(line):
            num_new_motions += have_moved
            pose = numpy.array([np.float64(line[1]), np.float64(line[2]), np.float64(line[3])])
            u = odom_control_gen.calculate_u(pose)

            have_moved = numpy.linalg.norm(u[:2]) > 1e-6
            first_obs_at_pos = first_obs_at_pos or have_moved
                

            for p in pc.particles: 
                mm.update(p, u)
            
            print "motion : {}".format(u)

        if isobservation(line):
           #  combine 1.1. and 1.2 as P(Z |X) = func(map_obj, cur_pose)
            laser_pose_offset = (np.float64(line[4]) - np.float64(line[1]), 
                                 np.float64(line[5]) - np.float64(line[2]), 
                                 np.float64(line[6]) - np.float64(line[3]))

            laser = [ ]
            laser_start = 7
            n_in_wall = 0
            for i in range(180):
                laser.append(np.float64(line[i + laser_start]))
            obs_view.vis_pose_and_laser(pose, laser)
            

            if first_obs_at_pos:
                num_new_observations += first_obs_at_pos
                first_obs_at_pos = False

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


        elif not ismotion(line):
            raise RuntimeError("unknown line type!!!11!!!1")

        if (num_new_motions > 0) and (num_new_observations > 0):
            num_new_motions = 0
            num_new_observations = 0
            pc.resample()
            #update stuff

        pc.show()
        

if __name__ == '__main__':
    numpy.random.seed(seed=7111990)
    pdbw.pdbwrap(main)()
