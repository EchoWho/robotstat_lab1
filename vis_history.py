import os, sys

import particle_filter
import numpy
import map_parser
import matplotlib.pyplot as plt

def vis_collection(fn, save = True, save_dir = None):
    collection = numpy.load(fn)['arr_0']

    if save_dir is None:
        save_dir = '{}_rendered/'.format(os.path.basename(fn))

    if not os.path.isdir(save_dir):
        os.mkdir(save_dir)
    else:
        raise RuntimeError("remove the old rendering")
    
    map_file = 'data/map/wean.dat'
    mo = map_parser.map_obj(map_file)

    fig = plt.figure(num = 1, figsize = (20, 20))
    pc = particle_filter.particle_collection(10, mo, fig_handle = fig)
    
    for entry_idx in range(collection.shape[2]):
        pose_coords = collection[:,:, entry_idx]

        print "plotting entry ", entry_idx
        pc.show(x = pose_coords[:, 0], y = pose_coords[:, 1])
        plt.savefig('{}/entry_{}.png'.format(save_dir, entry_idx))
        
    cmd = 'ffmpeg -start_number 0 -i {}/entry_%d.png -vcodec mpeg4 -y {}/movie.avi'.format(save_dir,
                                                                                           save_dir)
    os.system(cmd)

if __name__ == '__main__':
    fn = sys.argv[1]
    print fn
    vis_collection(fn)
