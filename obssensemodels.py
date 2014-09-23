import os, sys, pdb, numpy
from scipy import signal as signal
from scipy import stats as stats
from math import sqrt
import pdb

def mean_unif(a,b):
    return (a + b) / 2

def std_unif(a,b):
    return (b - a) / sqrt(12.0)

class observation_model:
    sigma = 5 # stdev of gaussian for p_hit (comp1_gauss)
    dmu = 0 # bias; distance from expected signal -- used in gaussian for p_hit (comp1_gauss)
    mu_expon = 0 # mean of exponential distribution
    spread_expon = 3
    max_rng = [48 , 52]
    # Relative weights of observation model components
    c = []
    c_hit = 13.0/16
    c_short = 2.0/16
    c_max = 1.0/16
    c_rand = 1.0/16
    """Some sort of map lookup thingy"""
    def Get_z_expected(self, x):
        return x
    def Get_p_z_given_x_u(self, z, x, u):
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
        

class motion_model:
    i = 12345
    p0 = 1 # uniform
    mu = [0,0]
    Sigma = [[1,0],[0,1]]

    def Get_p_x1_given_x0_u(self, x0, u):
         mu_x1 = mu + x0 + u
         ellipse = multivariate_normal(mean=mu, cov=Sigma)
         ellipse.pdf(x0)
         # We may need to add a uniform component to this

