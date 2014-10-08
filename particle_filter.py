import os, sys, pdb, signal, time

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
import matplotlib.cm
from numpy.random import multivariate_normal
import vis_history

# from multiprocessing import Pool

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
        hm = self.map_obj.hit_map.copy()
        self.canvas = 255 * numpy.dstack((numpy.zeros_like(hm), hm, hm)).astype('uint8')
        self.xy_record = None

        plt.figure(1)
#        plt.subplot(211)
        imgplot = plt.imshow(1 - map_orient, cmap = matplotlib.cm.gray)
        plt.show(block = False)

        # for i in range(1):
        #    delt= multivariate_normal(mean = np.array([0,0,0]), 
        #        cov = numpy.diag([50, 50, 10 / 180.0 * numpy.pi]))
        #    self.particles.append(particle(numpy.array([3975, 
        #                                                4100, 
        #                                                numpy.pi]), 1.0))

        for p_idx in range(n_particles):
          pos_idx = int(numpy.random.uniform(0, num_pos - 1e-6))
          pos = vec_pos[pos_idx]
          theta_i = int(numpy.random.uniform(0, nbr_theta - 1e-6))
          self.particles.append(particle(numpy.array([self.map_obj.resolution * pos[0], 
                                                      self.map_obj.resolution * pos[1], 
                                                      theta_i * unit_theta]),
                                         1.0))

    def record_xy(self):
        print "recording xy"
        pose_coords = numpy.asarray([self.map_obj.get_pose_coord(p.pose) for p in self.particles])
        if self.xy_record is None:
            self.xy_record = pose_coords
        else:
            self.xy_record = numpy.dstack((self.xy_record, pose_coords))

        print "record shape: {}".format(self.xy_record.shape)
        return pose_coords

        # numpy.random.shuffle(vec_pos)
        # random_positions = vec_pos[:n_particles]
        # random_positions = numpy.random.choice(vec_pos, size = n_particles,
        #                                        replace = False)

        # print "created {} particles".format(len(self.particles))

    def show(self, x = None, y = None):
        if self.last_scatter is not None:
            self.last_scatter.remove()

        canvas = self.canvas.copy()

        color = [255, 128, 0]
        
        if x is None and y is  None:
            pose_coords = self.record_xy()
        
            x = pose_coords[:, 0]
            y = pose_coords[:, 1]
        
        self.plot_xy(x, y)
        
    def plot_xy(self, x, y):

        plt.figure(1,figsize = (20, 20))
#        plt.subplot(211)
        self.last_scatter = plt.scatter(x, y, s = 10, c='red', marker='o', edgecolors='none')
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

        angle_var = 10 * numpy.pi / 180
        angle_vars = numpy.random.normal(loc = 0, scale = angle_var**2, size = M)
        
        for i in range(M):
          selected.append(copy.deepcopy(self.particles[idx]))
          selected[-1].weight = 1        
          selected[-1].pose[2] += angle_vars[i]
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

    fig = plt.figure(num = 1, figsize = (10, 10))

#    sf1 = fig.add_subplot(2,1,1)
#    sf2 = fig.add_subplot(2,1,2)
    # thread_pool = Pool(16)

    map_file = 'data/map/wean.dat'

    mo = map_parser.map_obj(map_file)
    logfile_fn = 'data/log/robotdata1.log'

    import datetime
    ts = str(datetime.datetime.now()).split()[1]
    record_fn = 'xy_record_{}_{}.npz'.format(os.path.basename(logfile_fn), ts)
    

    log = logparse.logparse(logfile_fn)
    
    n_particles = 5000
    print "creating particle collection of {} particles".format(n_particles)
    pc = particle_collection(n_particles = n_particles,
                             map_obj = mo,
                             nbr_theta = 360,
                             fig_handle = fig)

    print "created particle collection"

    have_moved = True
    first_obs_at_pos = True
    num_new_motions = 0
    num_new_observations = 0
    
    odom_control_gen = motion_model.odometry_control_generator()

    
    mm = motion_model.motion_model()
    obs_model = obssensemodels.observation_model(map_obj = mo, cpp_motion_model = mm.cpp_motion_model)
    obs_view = obssensemodels.observation_view(fig_handle = fig, map_obj = mo,
                                               cpp_map_obj = obs_model.cpp_map_obj)

    #mo.show()
    #print "showing pc"
    pc.show()

    pose = pc.particles[0].pose

    # changing cpp params won't affect these...
    # mo.vis_z_expected(pose)
    # obs_model.vis_p_z_given_x_u(pose)
    # pdb.set_trace()
    
    #todo remove start idx
    for (l_idx, line) in enumerate(log.lines):
        line = line.split()

        print "line {} / {}".format(l_idx + 1, len(log.lines))

        if ismotion(line): #or isobservation(line):
            num_new_motions += have_moved
            pose = numpy.array([np.float64(line[1]), np.float64(line[2]), np.float64(line[3])])
            u = odom_control_gen.calculate_u(pose)

            have_moved = numpy.linalg.norm(u[:2]) > 1e-6
            first_obs_at_pos = first_obs_at_pos or have_moved
            
            u_norm = numpy.linalg.norm(u[:2])
            u_arctan = numpy.arctan2(u[1], u[0])

            print "computing motion model.."
            for p in pc.particles: 
                p.pose[2] = 7 * 2 * numpy.pi / 8
                mm.update(p, u, u_norm, u_arctan)
                #pass

        if isobservation(line):
           #  combine 1.1. and 1.2 as P(Z |X) = func(map_obj, cur_pose)
            laser_pose_offset = (np.float64(line[4]) - np.float64(line[1]), 
                                 np.float64(line[5]) - np.float64(line[2]), 
                                 np.float64(line[6]) - np.float64(line[3]))
            offset_norm = numpy.linalg.norm(laser_pose_offset[:2])
            offset_arctan = numpy.arctan2(laser_pose_offset[1], laser_pose_offset[0])

            laser = [ ]
            laser_start = 7
            n_in_wall = 0
            for i in range(180):
                laser.append(np.float64(line[i + laser_start]))

            print numpy.max(laser)

            #pose = pc.particles[0].pose
            #mo.vis_z_expected(pose)
            #obs_view.vis_pose_and_laser(pose, laser)
            #print (pose)

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
                print "updating weights..."


                # pdb.set_trace()

                use_cpp_version = True
                do_both = False
                py_weights = []

                if use_cpp_version or do_both:
                    print "using cpp version"
                    poses = numpy.array([p.pose.copy() for p in pc.particles])

                    update_particle_weights_func = obs_model.cpp_observation_model.update_particle_weights
                    
                    weights = update_particle_weights_func(poses,
                                                           numpy.array(laser_pose_offset, 
                                                                       dtype = numpy.float64),
                                                           numpy.array([offset_norm, offset_arctan], 
                                                                       dtype=numpy.float64),
                                                           numpy.array(laser, dtype = numpy.float64))

                    if (weights.shape != (len(pc.particles),)):
                        raise RuntimeError("cpp weights wrong dim!")
                    for (p_idx, p) in enumerate(pc.particles):
                        p.weight *= weights[p_idx]                
                if not use_cpp_version or do_both:
                    print "using python version"
                    for p_idx, p in enumerate(pc.particles):
                        one_weight = obs_model.get_weight(p.pose, 
                                                          laser_pose_offset, 
                                                          offset_norm, 
                                                          offset_arctan, 
                                                          laser)
                        py_weights.append(one_weight)
                        p.weight *= one_weight
                    
                    # py_weights = numpy.array(py_weights)
                    # pdb.set_trace()

                new_weights = pc.get_weights()
                print "max weight: {}".format(new_weights.max())
                max_pose = pc.particles[np.argmax(new_weights)].pose 
                print "max weight location: {}".format( max_pose )
                pose_debug = np.array([ 3975, 4130, numpy.pi ])
                print "weight of {} is {} ".format( pose_debug, obs_model.get_weight(pose_debug, laser_pose_offset, offset_norm, offset_arctan, laser))
                obs_view.vis_pose_and_laser(max_pose, laser)
                # pdb.set_trace()
                #obs_view.vis_pose_and_laser(pose_debug, laser)
                
                #max_pose_new = max_pose
                #max_pose_new[2] -= numpy.pi/2
                #pose_debug_new = pose_debug
                #pose_debug_new[2] -= numpy.pi/2
                #vw1 = obs_model.get_vec_point_wise_weight(max_pose_new, laser)
                #vw2 = obs_model.get_vec_point_wise_weight(pose_debug_new, laser)
                #pdb.set_trace()

        elif not ismotion(line):
            raise RuntimeError("unknown line type!!!11!!!1")

        if (num_new_motions > 0) and (num_new_observations > 0):
            num_new_motions = 0
            num_new_observations = 0
            
            print "resampling..."

            try:
                pc.resample()
            except AssertionError:
                vis_history.vis_collection(record_fn)                
            print "resampled"
            #update stuff
        
        print "lidx ", l_idx
        display_period = 8
        if l_idx % display_period == 0:
            print "updating display..."
            pc.show()
            print "updated"
        if l_idx % 1 == 0:
            numpy.savez_compressed(record_fn, pc.xy_record)

    vis_history.vis_collection(record_fn)

def signal_handler(signal, frame):
    time.sleep(2)
    sys.exit()

if __name__ == '__main__':
    numpy.random.seed(seed = 7111990)
    signal.signal(signal.SIGINT, signal_handler)        
    pdbw.pdbwrap(main)()
