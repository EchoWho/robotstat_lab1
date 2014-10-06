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

        deg_sigma = 0.0
        self.motion_variance = 0
        self.Sigma = numpy.array([self.motion_variance, self.motion_variance, numpy.pi / 180. * deg_sigma]) * numpy.eye(3)

        self.alpha1 = 1e-3
        self.alpha2 = 1e-2
        self.alpha3 = 1e-2
        self.alpha4 = 1e-8

    def get_rot_mat(self, pose):
        theta = pose[2]
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        rot_mat = np.array([[cos_theta, -sin_theta, 0], 
                            [sin_theta, cos_theta, 0], 
                            [0,0,1]])
        return rot_mat

    def update_pose_with_sample(self, pose, sample):
        xnew = pose[0] + sample[1] * numpy.cos(pose[2] + sample[0])
        ynew = pose[1] + sample[1] * numpy.sin(pose[2] + sample[0])
        th_new = (pose[2] + sample[0] + sample[2]) % (2 * numpy.pi)
        
        new_pose = numpy.asarray([xnew, ynew, th_new], dtype = numpy.float64)
        return new_pose
        
    def update(self, x0, u):
        pose = x0.pose.copy()

        drot1 = numpy.arctan2(u[1], u[0]) - pose[2]
        dtrans = numpy.linalg.norm(u[:2])
        drot2 = u[2] - drot1


        
        drot1_sq = drot1**2
        dtrans_sq = dtrans**2
        drot2_sq = drot2**2

        sigma = numpy.array([self.alpha1 * drot1_sq + self.alpha2 * dtrans_sq,
                             self.alpha3 * dtrans_sq + self.alpha4 * drot1_sq + self.alpha4 * drot2_sq,
                             self.alpha1 * drot2_sq + self.alpha2 * dtrans_sq]) * \
            numpy.eye(3)
        
        
        sample = multivariate_normal(mean = [drot1, dtrans, drot2],
                                     cov = sigma)
        
        new_pose = self.update_pose_with_sample(pose, sample)
        x0.pose = new_pose

        # rot_mat = self.get_rot_mat(x0.pose)

        # u_world = rot_mat.dot(u + self.mu)
        # mu_x1 = x0.pose + u_world

        # sigma = self.Sigma.copy()
        
        # u_sigma = 0
        # sigma[range(3), range(3)] += u_sigma * u_world

        # sample = multivariate_normal(mean = mu_x1, cov=sigma)

        if 0 and numpy.linalg.norm(u[:2]) > 0:
            n_samples = 1000
            samples = multivariate_normal(mean = [drot1, dtrans, drot2], cov = sigma, size = n_samples)
            
            new_poses = []
            for s in range(n_samples):
                sample = samples[s, :]
                new_pose = self.update_pose_with_sample(pose, sample)
                new_poses.append(new_pose)
            new_poses = numpy.array(new_poses)

            fig = plt.figure()


            dx = 1e-2  * numpy.cos(new_poses[:, 2])
            dy = 1e-2 * numpy.sin(new_poses[:, 2])
            for s in range(n_samples):
                fig.hold()
                plt.plot([new_poses[s, 0], new_poses[s, 0] + dx[s]],
                         [new_poses[s, 1], new_poses[s, 1] + dy[s]])
            fig.hold()
            plt.scatter(new_poses[:, 0], new_poses[:, 1], color = 'k')

            # plt.scatter(pose[0], pose[1], color = 'r')
            plt.plot([pose[0], pose[0] + 1e-2 * numpy.cos(pose[2])],
                     [pose[1], pose[1] + 1e-2 * numpy.sin(pose[2])])
            

            plt.show(block = False)
            pdb.set_trace()



        # x0.pose = sample
        # x0.pose[-1] = x0.pose[-1] % (2 * numpy.pi)
        # print "u: {}, x0 pose: {}".format(u, x0.pose)
        # We may need to add a uniform component to this
