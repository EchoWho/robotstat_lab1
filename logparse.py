import os, sys, pdb, numpy

def calculate_u(odom_arr, idx_first, idx_second):
    obs_first = odom_arr[idx_first, :]
    obs_second = odom_arr[idx_second, :]
    u = obs_second - obs_first
    return u

# def main():
#     fn = 'log/robotdata1.log'
#     lines = open(fn, 'r').readlines()
#     odoms = {}
    
#     for (count, line) in enumerate(lines):
#         if line[0] == 'O':
#             odoms[count] = line.rstrip().split(' ')[1:]

#     print "entry length: {} ".format(len(odoms.values()[0]))
#     odom_arr = numpy.zeros((len(odoms), len(odoms.values()[0])), dtype = numpy.float64)
    
#     for (idx, (k, obs)) in enumerate(odoms.items()):
#         odom_arr[idx, :] = obs

#     u_s = numpy.zeros_like(odom_arr)
#     for t in range(odom_arr.shape[0] -1):
#         u = calculate_u(odom_arr, t, t+1)
#         u_s[t, :] = u

#     print "max x, min x: {}".format((odom_arr[:, 0].max(), odom_arr[:, 0].min()))
#     print "max y, min y: {}".format((odom_arr[:, 1].max(), odom_arr[:, 1].min()))
#     print "mean x: {}".format(odom_arr[:, 1].mean())
#     print "max theta, min theta: {}".format((odom_arr[:, 2].max(), odom_arr[:, 2].min()))

#     print "xdiff: {}".format(odom_arr[:, 0].sum())

#     pdb.set_trace()

class logparse(object):
    def __init__(self, fn = 'data/log/robotdata1.log'):
        lines = open(fn, 'r').readlines()
        odoms = {}
        ranges = []

        for (count, line) in enumerate(lines):
            if line[0] == 'O':
                odoms[count] = line.rstrip().split(' ')[1:]
            elif line[0] == 'L':
                ranges.append(map(float, line.split()[7:-1]))
                #parse lines

        print "entry length: {} ".format(len(odoms.values()[0]))
        odom_arr = numpy.zeros((len(odoms), len(odoms.values()[0])), dtype = numpy.float64)

        for (idx, (k, obs)) in enumerate(odoms.items()):
            odom_arr[idx, :] = obs

        u_s = numpy.zeros_like(odom_arr)
        for t in range(odom_arr.shape[0] -1):
            u = calculate_u(odom_arr, t, t+1)
            u_s[t, :] = u

        print "max x, min x: {}".format((odom_arr[:, 0].max(), odom_arr[:, 0].min()))
        print "max y, min y: {}".format((odom_arr[:, 1].max(), odom_arr[:, 1].min()))
        print "mean x: {}".format(odom_arr[:, 1].mean())
        print "max theta, min theta: {}".format((odom_arr[:, 2].max(), odom_arr[:, 2].min()))

        print "xdiff: {}".format(odom_arr[:, 0].sum())

        self.ranges = numpy.array(ranges)
        self.odom_arr = odom_arr
        self.u_s = u_s
        self.lines = lines

def main():
    lp = logparse()
    pdb.set_trace()

if __name__ == '__main__':
    main()
