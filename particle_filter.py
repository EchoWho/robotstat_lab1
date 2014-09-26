import os, sys

import map_parser
import logparse
import motion_model

def isobservation(line):
    return line[0] == 'L'

def ismotion(line):
    return line[0] == 'O'

class particle(object):
    def __init__(self, pose, weight):
        self.pose = pose
        self.weight = weight

class particle_collection(object):
    def __init__(self, n_particles, map_obj):
        self.map_obj = map_obj
        self.n_particles = n_particles
        self.particles = []

        #sample from map object the valid coordinates
        # create particles somehow across the thetas

    def get_weights(self):
        return numpy.array([p.weight for p in self.particles])

    def resample(self):
        weights = self.get_weights()
        w_sum = weights.sum()

        #do it

def main():

    map_file = '_'
    mo = map_parser.map_obj(map_file)

    logfile_fn = 'data/log/robotdata1.log'
    log = logparse.logparse(logfile_fn)
    
    
    n_particles = 100
    particle_collection = particle_collection(n_particles = n_particles,
                                              map_obj = mo)

    num_new_motions = 0
    num_new_observations = 0
    
    motion_model = motion_model.motion_model()


    for line in log.lines:
        if isobservation(line):
            num_new_observations += 1
            #weights change
        elif ismotion(line):
            num_new_motions += 1

            u = 1#something not 1
            for p in particle_collection.particles: 
                motion_model.update(p, u)
            #weights don't change
        else:
            raise RuntimeError("unknown line type!!!11!!!1")

        if (num_new_motions > 0) and (num_new_observations > 0):
            num_new_motions = 0
            num_new_observations = 0
            particle_collection.resample()
            #update stuff

if __name__ == '__main__':
    main()
