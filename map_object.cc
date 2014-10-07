#include "map_object.h"
using std::cout; 
using std::endl;

map_object::map_object()  { fake_init = true;}

map_object::map_object(boost::python::dict map_obj_dict)
{
    fake_init = false;
    ray_lookup = boost::python::extract<pyarr<double> >(map_obj_dict["ray_lookup"]);
    grid = boost::python::extract<pyarr<double> >(map_obj_dict["grid"]);
    hit_thresh  = 0.8;
    resolution = boost::python::extract<double>(map_obj_dict["resolution"]);
    n_angle_bins = boost::python::extract<int>(map_obj_dict["n_angle_bins"]);
    angle_bins_step = boost::python::extract<double>(map_obj_dict["angle_bins_step"]);

    cout << "resolution : " << resolution << endl;

    coord_idx_lookup = boost::python::extract<boost::python::dict>(map_obj_dict["coord_idx_lookup"]);

    o = boost::python::object(map_obj_dict);
    cout << "dictionary length: " << len(coord_idx_lookup) << endl;
}

void map_object::get_pose_coord(pyarr<double> &pose, int &coord0, int &coord1)
{
    coord0 = int((pose[ind(0)] / resolution) + .5);
    coord1 = int((pose[ind(1)] / resolution) + .5);
}

bool map_object::is_hit(int coord0, int coord1)
{
    if (0 <= coord0 && coord0 < grid.dims[0] && 
	0 <= coord1 && coord1 < grid.dims[1])
	return grid[ind(coord0, coord1)] <= hit_thresh;
    return true;
}

int map_object::lookup_ind_for_pose(pyarr<double> &pose)
{
    int coord0, coord1;
    get_pose_coord(pose, coord0, coord1);
    int idx = lookup_ind_for_coord(coord0, coord1);
    return idx;
}

int map_object::lookup_ind_for_coord(int &coord0, int &coord1)
{
    return 0;
    // boost::python::tuple t = boost::python::make_tuple(coord0, coord1);
    // if (!coord_idx_lookup.has_key(t))
    // {
    // 	cout << "idx: " << coord0 << " " << coord1 << endl;
    // 	cout << "length: " << len(coord_idx_lookup) << endl;
    // 	throw std::runtime_error("coord not found!");
    // }
    // return boost::python::extract<int>(coord_idx_lookup.get(t));
}

double map_object::get_z_expected(pyarr<double> &pose)
{
    int coord0, coord1;
    get_pose_coord(pose, coord0, coord1);
    int idx = lookup_ind_for_coord(coord0, coord1);

    double angle_mod = true_mod(pose[ind(2)], M_PI * 2);
    int angle_bin_idx = true_mod(int((angle_mod / angle_bins_step) + .5),
				 n_angle_bins);

    assert(0<= idx && idx < ray_lookup.dims[0] &&
	   0<= angle_bin_idx && angle_bin_idx < ray_lookup.dims[1]);

    return ray_lookup[ind(idx, angle_bin_idx)];
}



void boost_map_object()
{
    class_<map_object>("map_object", init<boost::python::dict>())
	.def_readonly("hit_thresh", &map_object::hit_thresh)
	.def_readonly("resolution", &map_object::resolution)
	;
}

BOOST_PYTHON_MODULE(libmap_object)
{
    import_array();
    boost_common();

    boost_map_object();
}

