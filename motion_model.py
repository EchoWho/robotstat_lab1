from numpy.random import multivariate_normal
import pdb
import math
import numpy
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

class odometry_control_generator(object):
    def __init__(self):
      self.last_odom = None

    def calculate_u(self, new_pose):
        assert(new_pose != None)

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
        self.motion_variance = 1e-6
        self.Sigma = numpy.array([self.motion_variance, self.motion_variance, numpy.pi / theta_precision]) * numpy.eye(3)

    def update(self, x0, u):
        gamma = x0.gamma 
        cos_gamma = math.cos(gamma)
        sin_gamma = math.sin(gamma)
        rot_mat = np.array([[cos_gamma, -sin_gamma, 0], [sin_gamma, cos_gamma, 0], [0,0,1]])

        u_world = rot_mat.dot(u + self.mu)
        mu_x1 = x0.pose + u_world

        sigma = self.Sigma.copy()
        sigma[range(2), range(2)] += .1 * u_world[:2]

        sample = multivariate_normal(mean = mu_x1, cov=sigma)

        if 0 and numpy.linalg.norm(u[:2]) > 0:
            samples = multivariate_normal(mean = mu_x1[:2], cov=self.Sigma[:2, :2], size = 1000)
            plt.figure()
            plt.scatter(samples[:, 0], samples[:, 1])
            plt.show(block = False)
            pdb.set_trace()

        x0.pose = sample
        x0.pose[-1] = x0.pose[-1] % (2 * numpy.pi)
        # print "u: {}, x0 pose: {}".format(u, x0.pose)
         # We may need to add a uniform component to this
