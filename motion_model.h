#pragma once

#include "util.h"

class motion_model{
public:
motion_model(float alpha1, float alpha2, float alpha3, float alpha4)
    : alpha1(alpha1) , alpha2(alpha2), alpha3(alpha3), alpha4(alpha4) {}
    

    motion_model() {}

    double alpha1;
    double alpha2;
    double alpha3;
    double alpha4;

    pyarr<double> update(pyarr<double> pose,
			 pyarr<double> u,
			 float u_norm,
			 float u_arctan);

    void compute_relative_transform(pyarr<double> &pose,
				    pyarr<double> &u,
				    float u_norm,
				    float u_arctan,
				    double &drot1,
				    double &dtrans,
				    double &drot2);

    boost::python::list py_update_pose_with_sample(pyarr<double> pose,
						   pyarr<double> sample);

    boost::python::list compute_relative_transform_float(pyarr<double> pose,
							 pyarr<double> u,
							 float u_norm,
							 float u_arctan);

    vector<double> update_pose_with_sample(pyarr<double> &pose,
					   vector<double> &sample);
};
