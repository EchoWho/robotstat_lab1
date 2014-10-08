import os, sys, pdb, numpy
from scipy import signal as signal
from scipy import stats as stats
import pdb
import map_parser
import matplotlib.pyplot as plt
import matplotlib.pyplot as plt
import matplotlib
import numpy as np
import gtkutils.img_util as iu
import motion_model
import copy

import warnings

with warnings.catch_warnings():
    warnings.simplefilter("ignore")
    import libobservation_model as lom
    import libmap_object as lmo


def mean_unif(a,b):
    return (a + b) / 2

def std_unif(a,b):
    return (b - a) / numpy.sqrt(12.0)

class observation_view(object):
    def __init__(self, fig_handle, map_obj, cpp_map_obj):
        self.fig_handle = fig_handle
        self.lastscatter = None
        self.map_obj = map_obj
        self.cpp_map_obj = cpp_map_obj

    def vis_pose_and_laser(self, pose, laser):
        canvas = numpy.zeros((800, 800), dtype = numpy.uint8)

        #center = (400, 400)
        th_offset = pose[2] - numpy.pi / 2.0
        x_data = []
        y_data = []
        x_expected = []
        y_expected = []

        for (zi, z) in enumerate(laser):
            th = zi * numpy.pi / 180.0
            x_data.append(numpy.cos(th) * z)
            y_data.append(numpy.sin(th) * z)

        pose_new = pose
        pose_new[2] -= numpy.pi / 2.0
        for zi in range(360):
            z_expected = self.map_obj.get_z_expected(pose_new)
            cpp_z_expected = self.cpp_map_obj.py_get_z_expected(pose_new)
            if abs(z_expected - cpp_z_expected) > 1e-4:
                raise RuntimeError("NO WTF")

            th = zi * numpy.pi / 180.0
            x_expected.append(numpy.cos(th) * z_expected)
            y_expected.append(numpy.sin(th) * z_expected)

            pose_new[2] += 1.0 / 180.0 * numpy.pi
            
        data = numpy.array([x_data, y_data])
#        pdb.set_trace()
        plt.figure(2)
        if self.lastscatter is not None:
            self.lastscatter.remove()
            self.lastexpected.remove()
        f1 = plt.gca()
#        plt.subplot(212)
        f1.clear()
        self.lastscatter = plt.scatter(data[0, :], data[1, :], c='blue', edgecolors='none')
        self.lastexpected = plt.scatter(np.array(x_expected), np.array(y_expected), c='red', edgecolors='none')

        bound = 1200
        plt.axis([-bound, bound, -bound, bound])
        plt.draw()

        #pdb.set_trace()

        

class observation_model:
    
    def __init__(self, map_obj, cpp_motion_model):


      self.mu_expon = 0 # mean of exponential distribution
      self.spread_expon = 10

      self.sigma = 40 # stdev of gaussian for p_hit (comp1_gauss) in centimeters
      self.sigma2 = self.sigma**2
      self.norm_const = 1/(self.sigma * numpy.sqrt(2*numpy.pi))

      self.min_rng = 100.0
      self.max_rng = 1200.0

      # Relative weights of observation model components
      self.c_hit = 4.0
      self.c_short = 0.1
      self.c_max = 0.5
      self.c_rand = 1.0 # keep this fixed.
      
      self.map_obj = map_obj

      self.sample_perc = .5

      self.ref_maintainer = copy.deepcopy(map_obj.__dict__)
      self.cpp_map_obj = lmo.map_object(self.ref_maintainer)


      self.cpp_observation_model = lom.observation_model(self.c_hit, self.c_rand, self.c_short, self.c_max,
                                                         self.max_rng, self.sigma,
                                                         self.cpp_map_obj, cpp_motion_model)

      # self.cpp_observation_model = lom.observation_model(self.cpp_map_obj, cpp_motion_model)

      self.cpp_observation_model._check_lookup_size()
      self.cpp_motion_model = cpp_motion_model
      
    def get_rot_mat(self, pose):
        theta = pose[2]
        cos_theta = numpy.cos(theta)
        sin_theta = numpy.sin(theta)
        rot_mat = numpy.array([[cos_theta, -sin_theta, 0], 
                            [sin_theta, cos_theta, 0], 
                            [0,0,1]])
        return rot_mat

    def get_weight(self, pose, laser_pose_offset, offset_norm, offset_arctan, laser):

        # print "pose new", pose_new
        # pose_coord = self.map_obj.get_pose_coord(pose_new)
        # print "pose coord", pose_coord
        # print "z exp", self.map_obj.get_z_expected(pose_new)

        # weight = self.cpp_observation_model.get_weight(pose.copy(),
        #                                                numpy.array(laser_pose_offset, dtype = numpy.float64),
        #                                                offset_norm.item(),
        #                                                offset_arctan.item(),
        #                                                numpy.array(laser, dtype = numpy.float64))

        ## used to sanity check the cpp
        # l = self.cpp_motion_model.compute_relative_transform_float(pose, 
        #                                                  numpy.array(laser_pose_offset,
        #                                                              dtype = numpy.float64),
        #                                                  offset_norm.item(), 
        #                                                  offset_arctan.item())
        # cpp_pose_new = self.cpp_motion_model.py_update_pose_with_sample(pose.copy(), 
        #                                                                 numpy.array([drot1, dtrans, drot2],
        #                                                                             dtype = numpy.float64))


        
        drot1, dtrans, drot2 = motion_model.compute_relative_transform(pose, 
            laser_pose_offset, offset_norm, offset_arctan)

        pose_new = motion_model.update_pose_with_sample(pose, [drot1, dtrans, drot2])

        pose_new[2] -= numpy.pi / 2.0 


        
        # if the Laser pose is in the wall then the particle has weight 0
        if self.map_obj.is_hit(pose_new) or self.map_obj.is_hit(pose):
            return 0

        
        # return self.get_point_wise_weight(pose_new, laser)
        return self.get_func_inner_product_weight(pose_new, laser)
       
    def get_func_inner_product_weight(self, pose_new, laser):
        delt_theta = numpy.pi / 180.0
        
        sums = np.zeros(3, dtype=np.float64)
        for zi, z in enumerate(laser):
            z_exp = self.Get_z_expected(pose_new)
            sums[0] += z_exp * z
            sums[1] += z_exp * z_exp
            sums[2] += z * z
            pose_new[2] = (pose_new[2] + delt_theta) % (2 *numpy.pi)
        return sums[0] **2 / (sums[1] * sums[2])

    def get_point_wise_weight(self, pose_new, laser):
        delt_theta = numpy.pi / 180.0
        log_weight_sum = 0

        self.current_pw_weights = []
        self.current_pose_new_log = []

        for zi, z in enumerate(laser):
            if (zi % 5 == 0):
                logpz = self.Get_log_p_z_given_pose_u(z, pose_new)
                log_weight_sum += logpz
                self.current_pw_weights.append(logpz)
            pose_new[2] = (pose_new[2] + delt_theta) % (2 * numpy.pi)

        weight = numpy.exp(log_weight_sum)


        return weight

    def get_vec_point_wise_weight(self, pose_new, laser):
        delt_theta = numpy.pi / 180.0
        vec_weights = []
        for zi, z in enumerate(laser):
            vec_weights.append(numpy.exp(self.Get_log_p_z_given_pose_u(z, pose_new)))
            pose_new[2] = (pose_new[2] + delt_theta) % (2 * numpy.pi)

        return vec_weights



    def Get_z_expected(self, pose):
        z = self.map_obj.get_z_expected(pose)
        return z

    def Get_log_p_z_given_pose_u(self, z, pose):
        assert(len(pose) == 3)
        z_exp = self.Get_z_expected(pose)

        # Determine relative weights for each component in the observation model
        # Add in any parameter changes to the distribution based on u, z_expected
#        pdb.set_trace()
        C_hit = self.c_hit
        C_short = self.c_short
        C_max = self.c_max
        C_rand = self.c_rand
        # Normalize to 1 (probability distribution should integrate to 1)
        # uniform is assumed to be 1
        p_z_given_x = 1

        if (z > self.max_rng and z_exp > self.max_rng):
          p_z_given_x += C_max 

        # p_hit =  stats.norm.pdf(z, loc=(z_exp + self.dmu), scale=self.sigma) # comp1_gauss
        p_hit = self.norm_const * numpy.exp(-(z - z_exp)**2 / (2 * self.sigma2))

        #p_short = stats.expon.pdf(z, self.mu_expon, self.spread_expon)
        p_short =  1.0 / np.float64(z*1.55e-4+1.0) 

        # pdb.set_trace()
        p_z_given_x += C_hit * p_hit + C_short * p_short 
        
        return numpy.log(p_z_given_x)

    def vis_p_z_given_x_u(self, pose):
        data = []
        zs = numpy.arange(0, self.max_rng + 100, 10)
        for z in zs:
            pz = self.Get_log_p_z_given_pose_u(z, pose)
            data.append(numpy.exp(pz))
            
        f = plt.figure()
        p = plt.plot(zs, data)
        # plt.axis([0, 8000, 0, 8000])
        plt.show(block = False)
#        pdb.set_trace()
            

        

