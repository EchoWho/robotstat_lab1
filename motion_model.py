from numpy.random import multivariate_normal
import math
import numpy
import numpy as np

class odometry_control_generator(object):
    def __init__(self):
      self.last_odom = None

    def calculate_u(self, new_pose):
      if self.last_odom == None:
        u = numpy.array(( 0, 0, 0 ), dtype = numpy.float64)
      else:
        u = new_pose - self.last_odom  
      self.last_odom = new_pose
      return u
        
# TODO- mu, sig
class motion_model(object):
    def __init__(self):
        self.p0 = 1 # uniform
        self.mu = numpy.zeros((3,), dtype = numpy.float64)

        theta_precision = float(2**5)
        self.Sigma = numpy.array([5, 5, numpy.pi / theta_precision]) * numpy.eye(3)

    def update(self, x0, u):
        gamma = x0.gamma 
        cos_gamma = math.cos(gamma)
        sin_gamma = math.sin(gamma)
        rot_mat = np.array([[cos_gamma, -sin_gamma, 0], [sin_gamma, cos_gamma, 0], [0,0,1]])
        mu_x1 = x0.pose + rot_mat.dot(u + self.mu)
        sample = multivariate_normal(mean = mu_x1, cov=self.Sigma)
        x0.pose = sample
        x0.pose[-1] = x0.pose[-1] % (2 * numpy.pi)
        # print "u: {}, x0 pose: {}".format(u, x0.pose)
         # We may need to add a uniform component to this
