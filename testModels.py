from obssensemodels import *
import numpy as np
import matplotlib.pyplot as plt

omdl = observation_model()
xs = np.linspace(-10,100,1000)
pzgivzu = np.zeros((xs.size,1))

for i in range(0, xs.size):
    u = 0
    pzgivzu[i] = omdl.Get_p_z_given_x_u(xs[i], 10, u)

plt.figure()
plt.title('p_z_given_x')
plt.plot(xs,pzgivzu)
plt.show()

# Still need to test this out
mmodl = motion_model()
