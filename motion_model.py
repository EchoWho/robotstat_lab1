from numpy.random import multivariate_normal

class odometry_control_generator(object):
    def __init__(self):
      self.last_odom = None:

    def calculate_u(self, new_pose):
      if self.last_odom == None:
        u = ( 0, 0, 0 )
      else:
        u = new_pose - self.last_odom  
      self.last_odom = new_pose
      return u
        

class motion_model(object):
    def __init__(self):
        self.i = 12345
        self.p0 = 1 # uniform
        self.mu = [0,0]
        self.Sigma = [[1,0],[0,1]] * 0.1

    def update(self, x0, u):
        x1 = self.Get_p_x1_given_x0_u(x0, u)

    def Get_p_x1_given_x0_u(self, x0, u):
         mu_x1 = self.mu + x0 + u
         ellipse = multivariate_normal(mean=self.mu, cov=self.Sigma)
         ellipse.pdf(x0)
         # We may need to add a uniform component to this
