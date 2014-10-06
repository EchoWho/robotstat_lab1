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

def mean_unif(a,b):
    return (a + b) / 2

def std_unif(a,b):
    return (b - a) / numpy.sqrt(12.0)

class observation_view(object):
    def __init__(self, fig_handle):
        self.fig_handle = fig_handle
        self.lastscatter = None

    def vis_pose_and_laser(self, pose, laser):
        canvas = numpy.zeros((800, 800), dtype = numpy.uint8)

        #center = (400, 400)
        x_data = []
        y_data = []
        for (zi, z) in enumerate(laser):
            th = zi * numpy.pi / 180.0
            x_data.append(numpy.cos(th) * z)
            y_data.append(numpy.sin(th) * z)
            
            # try: 
            #     canvas[center[0] + x, center[0] + y] = 255
            # except IndexError:
            #     pass
            # print x,y
        
        data = numpy.array([x_data, y_data])
        # canvas.transpose()
#        pdb.set_trace()
        plt.figure(2)
        if self.lastscatter is not None:
            self.lastscatter.remove()
        f1 = plt.gca()
#        plt.subplot(212)
        f1.clear()
        self.lastscatter = plt.scatter(data[0, :], data[1, :])
        plt.axis([0, 8000, 0, 8000])
        plt.draw()
        

class observation_model:
    
    def __init__(self, map_obj):
      self.sigma = 50 # stdev of gaussian for p_hit (comp1_gauss) in centimeters
      self.sigma2 = self.sigma**2
      self.norm_const = 1/(self.sigma * numpy.sqrt(2*numpy.pi))
      self.dmu = 0 # bias; distance from expected signal -- used in gaussian for p_hit (comp1_gauss)
      self.mu_expon = 0 # mean of exponential distribution
      self.spread_expon = 10
      self.max_rng = [7000 , 8000] #need to calculate these

      # Relative weights of observation model components
      self.c_hit = 1.
      self.c_short = 1. 
      self.c_max = 1.
      self.c_rand = 100
      self.map_obj = map_obj
      # self.compute_normalizer()

    def get_rot_mat(self, pose):
        theta = pose[2]
        cos_theta = numpy.cos(theta)
        sin_theta = numpy.sin(theta)
        rot_mat = numpy.array([[cos_theta, -sin_theta, 0], 
                            [sin_theta, cos_theta, 0], 
                            [0,0,1]])
        return rot_mat

    def get_weight(self, pose, laser_pose_offset, laser):
        pose_offset_world = self.get_rot_mat(pose).dot(laser_pose_offset)

        pose_new = list(pose + pose_offset_world)
        pose_new[2] -= numpy.pi / 2.0 
        delt_theta = numpy.pi / 180.0
        
        # if the Laser pose is in the wall then the particle has weight 0
        if self.map_obj.is_hit(pose_new):
            return 0
            
        weight = np.float64(1)
        weight_sum = 0
        for zi, z in enumerate(laser):
            #weight *= self.Get_p_z_given_pose_u(z, pose_new)
            weight_sum += self.Get_p_z_given_pose_u(z, pose_new)
            pose_new[2] += (pose_new[2] + delt_theta) % (2 * numpy.pi)
        return weight_sum / np.float64(len(laser))
      
    def Get_z_expected(self, pose):
        z = self.map_obj.get_z_expected(pose)
        return z

    def Get_p_z_given_pose_u(self, z, pose):
        assert(len(pose) == 3)
        z_exp = self.Get_z_expected(pose)
        # Determine relative weights for each component in the observation model
        # Add in any parameter changes to the distribution based on u, z_expected
#        pdb.set_trace()
        C_hit = self.c_hit * 1
        C_short = self.c_short * 1
        C_max = self.c_max * 1
        C_rand = self.c_rand * 1
        # Normalize to 1 (probability distribution should integrate to 1)
        sum_Cs = C_hit + C_short + C_max + C_rand
        C_hit = C_hit / sum_Cs
        C_short = C_short / sum_Cs
        C_max = C_max / sum_Cs
        C_rand = C_rand / sum_Cs

        # p_hit =  stats.norm.pdf(z, loc=(z_exp + self.dmu), scale=self.sigma) # comp1_gauss
        p_hit =   self.norm_const * numpy.exp(-(z - z_exp)**2 / (2 * self.sigma2))

        # p_short = stats.expon.pdf(z, self.mu_expon, self.spread_expon)
        p_short =  1 / np.float64(z+1) 

        # unif1 = stats.uniform(loc = self.max_rng[0], scale = (self.max_rng[1] - self.max_rng[0]) )
        # p_max = unif1.pdf(z) # Uniform distribution

        unif1 = 1 / np.float64(self.max_rng[1] - self.max_rng[0])
        p_max = unif1 # Uniform distribution

        # unif2 = stats.uniform( loc = 0, scale = (self.max_rng[1] - 0) )
        # p_rand = unif2.pdf(z) # Uniform distr.

        unif2 =  1 / np.float64(self.max_rng[1])
        p_rand = unif2

        # pdb.set_trace()
        p_z_given_x = C_hit * p_hit + C_rand * p_rand# + C_short * p_short + C_max * p_max 
        
        return p_z_given_x

    def vis_p_z_given_x_u(self, pose):
        data = []
        zs = numpy.arange(0, 8000, 1)
        for z in zs:
            pz = self.Get_p_z_given_pose_u(z, pose)
            data.append(pz)
            
        f = plt.figure()
        p = plt.scatter(zs, data)
        p.axis([0, 8000, 0, 8000])
        plt.show(block = False)
        pdb.set_trace()
            
        

