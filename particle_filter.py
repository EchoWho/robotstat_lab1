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

        unit_theta = 2 * math.pi / float(nbr_theta)
        vec_pos = map_obj.get_valid_coordinates()

        numpy.random.shuffle(vec_pos)
        random_positions = vec_pos[:n_particles]
        # random_positions = numpy.random.choice(vec_pos, size = n_particles,
        #                                        replace = False)

        for pos in random_positions:
            for theta_i in range(nbr_theta):
                self.particles.append(particle(
                                               numpy.array([self.map_obj.resolution * pos[0], 
                                                            self.map_obj.resolution * pos[1], 
                                                            theta_i * unit_theta]),1.0))
        # for theta_i in range(nbr_theta):
        #   for pos in enumerate(vec_pos):

        print "created {} particles".format(len(self.particles))

    def show(self):
        hm = self.map_obj.hit_map.copy()
        canvas = 255 * numpy.dstack((numpy.zeros_like(hm), hm, hm)).astype('uint8')

        color = [255, 128, 0]
        
        for p in self.particles:
            pose_coord = self.map_obj.get_pose_coord(p.pose)

            canvas[pose_coord[0] - 1, pose_coord[1] - 1, :] = color
            canvas[pose_coord[0] - 1, pose_coord[1], :] = color
            canvas[pose_coord[0] - 1, pose_coord[1] + 1, :] = color

            canvas[pose_coord[0] + 1, pose_coord[1] + 1, :] = color
            canvas[pose_coord[0] + 1, pose_coord[1], :] = color
            canvas[pose_coord[0] + 1, pose_coord[1] - 1, :] = color

            canvas[pose_coord[0], pose_coord[1] - 1, :] = color
            canvas[pose_coord[0], pose_coord[1] + 1, :] = color

            canvas[pose_coord[0], pose_coord[1], :] = color
            
        iu.v(canvas)

    def get_weights(self):
        return numpy.array([p.weight for p in self.particles])

    def resample(self):
        #shuffle? shuffle.
        numpy.random.shuffle(self.particles)

        weights = self.get_weights()
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
          while w >= w_cumsums[idx]:
            idx += 1

        self.particles = selected
        

def main():

    map_file = 'data/map/wean.dat'

    mo = map_parser.map_obj(map_file)
    
    # mo.

    logfile_fn = 'data/log/robotdata1.log'
    log = logparse.logparse(logfile_fn)
    
    
    n_particles = 100
    print "creating particle collection of {} particles".format(n_particles)
    pc = particle_collection(n_particles = n_particles,
                             map_obj = mo)
    print "created particle collection"

    num_new_motions = 0
    num_new_observations = 0
    
    ocg = motion_model.odometry_control_generator()
    obs_model = obssensemodels.observation_model(map_obj = mo)
    mm = motion_model.motion_model()

    # mo.show()
    # pc.show()


    pose     = pc.particles[200].pose
    
    mo.vis_z_expected(pose)

    print pose
    obs_model.vis_p_z_given_x_u(pose)

    pdb.set_trace()
    
    for (l_idx, line) in enumerate(log.lines):
        line = line.split()

        print "line {} / {}".format(l_idx + 1, len(log.lines))

        if ismotion(line):
            num_new_motions += 1
            pose = numpy.array([np.float64(line[1]), np.float64(line[2]), np.float64(line[3])])
            u = ocg.calculate_u(pose)
            for p in pc.particles: 
                mm.update(p, u)

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
            n_in_wall = 0
            for i in range(180):
              laser.append(np.float64(line[i + laser_start]))
            for (p_idx, p) in enumerate(pc.particles):
                if (p_idx % mo.n_angle_bins) == 0:
                    mo.vis_z_expected(p.pose)                    

                p.weight *= obs_model.get_weight(p.pose, laser_pose_offset, laser)
                if p.weight == 0:
                    n_in_wall += 1
                print "on pidx {}/{}".format(p_idx + 1, len(pc.particles))

            if (n_in_wall > 0):
                mo.show()
                pc.show()
                
            
        else:
            raise RuntimeError("unknown line type!!!11!!!1")

        if (num_new_motions > 0) and (num_new_observations > 0):
            num_new_motions = 0
            num_new_observations = 0
            pc.resample()
            #update stuff

    print "DONE"

if __name__ == '__main__':
    numpy.random.seed(seed=7111990)
    pdbw.pdbwrap(main)()
