import os, sys, pdb, numpy
from scipy import signal as signal
from scipy import stats as stats
from math import sqrt
import pdb
import map_parser

def mean_unif(a,b):
    return (a + b) / 2

def std_unif(a,b):
    return (b - a) / sqrt(12.0)

class observation_model:
    
    def __init__(map_obj):
      sigma = 5 # stdev of gaussian for p_hit (comp1_gauss)
      dmu = 0 # bias; distance from expected signal -- used in gaussian for p_hit (comp1_gauss)
      mu_expon = 0 # mean of exponential distribution
      spread_expon = 3
      max_rng = [48 , 52] #need to calculate these
      # Relative weights of observation model components
      c = []
      c_hit = 13.0/16
      c_short = 2.0/16
      c_max = 1.0/16
      c_rand = 1.0/16
      self.map_obj = map_obj


    def get_weight(self, pose, laser_pose_offset, laser):
      x = pose + laser_pose_offset
      x[2] -= math.pi / 2.0 
      delt_theta = math.pi / 180.0

      # if the Laser pose is in the wall then the particle has weight 0
      if self.map_obj.is_hit(x):
        return 0

      weight = 1
      for zi, z in enumerate(laser):
        weight *= self.Get_p_z_given_x_u(z, x)
        x[2] += delt_theta
      return weight
      
    #Some sort of map lookup thingy
    def Get_z_expected(self, x):
      return self.map_obj.ray_finding(x)  


    def Get_p_z_given_x_u(self, z, x):
        z_exp = self.Get_z_expected(x)
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

        p_hit =  stats.norm.pdf(z, loc=(z_exp + self.dmu), scale=self.sigma) # comp1_gauss
        p_short = stats.expon.pdf(z, self.mu_expon, self.spread_expon) # comp2_exp = # Exponential distribution here
        unif1 = stats.uniform(loc = self.max_rng[0], scale = (self.max_rng[1] - self.max_rng[0]) )
        p_max = unif1.pdf(z) # Uniform distribution
        unif2 = stats.uniform( loc = 0, scale = (self.max_rng[1] - 0) )
        p_rand = unif2.pdf(z) # Uniform distr.
        # pdb.set_trace()
        p_z_given_x = C_hit * p_hit + C_short * p_short + C_max * p_max + C_rand * p_rand
        return p_z_given_x
        

