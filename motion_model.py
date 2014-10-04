from numpy.random import multivariate_normal
import numpy

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

        theta_precision = float(2**6)
        self.Sigma = numpy.array([0.5, 0.5, numpy.pi / theta_precision]) * numpy.eye(3)

    def update(self, x0, u):
        mu_x1 = self.mu + x0.pose + u
        sample = multivariate_normal(mean = mu_x1, cov=self.Sigma)
        x0.pose = sample
        x0.pose[-1] = x0.pose[-1] % (2 * numpy.pi)
        print "u: {}, xo pose: {}".format(u, xo.pose)
         # We may need to add a uniform component to this
