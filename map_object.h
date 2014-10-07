#pragma once
#include <boost_common.h>
#include <pyarr.h>
#include <cmath>

#include "util.h"

class map_object {
public:
    map_object();

    map_object(boost::python::dict map_obj_dict);
    void get_pose_coord(pyarr<double> &pose, int &coord0, int &coord1);
    bool is_hit(int coord0, int coord1);

    double get_z_expected(vector<double> &pose);
    int lookup_ind_for_coord(int &coord0, int &coord1);
    int lookup_ind_for_pose(pyarr<double> &pose);
    int lookup_ind_for_pose(vector<double> &pose);

    void get_pose_coord(vector<double> &pose, int &coord0, int &coord1);
    pyarr<double> get_grid();

    float hit_thresh;
    int resolution, n_angle_bins;
    double angle_bins_step;

    pyarr<double> ray_lookup;
    pyarr<double> grid;
    bool fake_init;

    boost::python::object o;
    boost::python::dict coord_idx_lookup;
    std::map<std::pair<int, int>, int> coord_idx_lookup2;
};

void boost_map_object();
