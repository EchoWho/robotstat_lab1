
class control_calculator(object):
    def __init__(self, odometry_pose):
        self.odometry_pose = odometry_pose

    def calculate_u(self, new_pose):
        pass
        
        

class motion_model(object):
    def __init__(self):
        self.i = 12345
        self.p0 = 1 # uniform
        self.mu = [0,0]
        self.Sigma = [[1,0],[0,1]]

    def update(self, x0, u):
        x1 = self.Get_p_x1_given_x0_u(x0, u)

    def Get_p_x1_given_x0_u(self, x0, u):
         mu_x1 = mu + x0 + u
         ellipse = multivariate_normal(mean=mu, cov=Sigma)
         ellipse.pdf(x0)
         # We may need to add a uniform component to this
