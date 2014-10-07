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
    cout << "hit thresh: " << hit_thresh << endl;

    boost::python::list items = coord_idx_lookup.items();
    for (size_t i =0; i < len(items); i++)
    {
	boost::python::object item = items[i];
	boost::python::tuple key = boost::python::extract<boost::python::tuple>(item[0]);
	int value = boost::python::extract<int>(item[1]);

	int keyp_0 = boost::python::extract<int>(key[0]);
	int keyp_1 = boost::python::extract<int>(key[1]);
	std::pair<int, int> key_p = std::make_pair(keyp_0, keyp_1);
	coord_idx_lookup2[key_p] = value;
    }
}

void map_object::get_pose_coord(pyarr<double> &pose, int &coord0, int &coord1)
{
    coord0 = int((pose[ind(0)] / resolution) + .5);
    coord1 = int((pose[ind(1)] / resolution) + .5);
}

void map_object::get_pose_coord(vector<double> &pose, int &coord0, int &coord1)
{
    coord0 = int((pose[0] / resolution) + .5);
    coord1 = int((pose[1] / resolution) + .5);
}


bool map_object::is_hit(int coord0, int coord1)
{
    if (0 <= coord0 && coord0 < grid.dims[0] && 
	0 <= coord1 && coord1 < grid.dims[1])
	return grid[ind(coord0, coord1)] <= hit_thresh;
    return true;
}

int map_object::lookup_ind_for_pose(vector<double> &pose)
{
    int coord0, coord1;
    get_pose_coord(pose, coord0, coord1);
    int idx = lookup_ind_for_coord(coord0, coord1);
    return idx;
}


int map_object::lookup_ind_for_coord(int &coord0, int &coord1)
{

    std::pair<int, int> key = std::make_pair(coord0, coord1);
    
    if (coord_idx_lookup2.find(key) == coord_idx_lookup2.end())
    {
	cout << "didnt find: " << coord0 << " " << coord1 << endl;
    	throw std::runtime_error("coord not found!");
    }
    
    return coord_idx_lookup2.find(key)->second;
}

// int map_object::lookup_ind_for_coord(int &coord0, int &coord1)
// {

//     boost::python::tuple t;
// #pragma omp critical (tuple)
//     t = boost::python::make_tuple(coord0, coord1);

// #pragma omp critical (xkey)
//     if (!coord_idx_lookup.has_key(t))
//     {
//     	cout << "idx: " << coord0 << " " << coord1 << endl;
//     	cout << "length: " << len(coord_idx_lookup) << endl;
//     	throw std::runtime_error("coord not found!");
//     }

// int idx;
// #pragma omp critical (lookup)
// idx = boost::python::extract<int>(coord_idx_lookup.get(t));

// return idx;
// }

double map_object::get_z_expected(vector<double> &pose)
{
    int coord0, coord1;
    get_pose_coord(pose, coord0, coord1);
    int idx = lookup_ind_for_coord(coord0, coord1);

    double angle_mod = true_mod(pose[2], M_PI * 2);
    int angle_bin_idx = true_mod(int((angle_mod / angle_bins_step) + .5),
				 n_angle_bins);

    assert(0<= idx && idx < ray_lookup.dims[0] &&
	   0<= angle_bin_idx && angle_bin_idx < ray_lookup.dims[1]);

    return ray_lookup[ind(idx, angle_bin_idx)];
}

pyarr<double> map_object::get_grid()
{
    return grid;
}

void boost_map_object()
{
    class_<map_object>("map_object", init<boost::python::dict>())
	.def_readonly("hit_thresh", &map_object::hit_thresh)
	.def_readonly("resolution", &map_object::resolution)
	.def("get_grid", &map_object::get_grid)
	;
}

BOOST_PYTHON_MODULE(libmap_object)
{
    import_array();
    boost_common();

    boost_map_object();
}

