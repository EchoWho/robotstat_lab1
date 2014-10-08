#include <boost_common.h>
#include <pyarr.h>
#include <cmath>

#include <boost/math/distributions/normal.hpp>
#include <boost/random.hpp>

#include "motion_model.h"

using std::cout;
using std::endl;


boost::random::normal_distribution<> normal_dist(0, 1);

boost::random::mt19937 rng;

boost::variate_generator<boost::mt19937&, 
			     boost::normal_distribution<> > normal_generator(rng, normal_dist);

boost::python::list motion_model::compute_relative_transform_float(pyarr<double> pose,
								   pyarr<double> u,
								   float u_norm,
								   float u_arctan)
{
    double drot1, dtrans, drot2;
    compute_relative_transform(pose,
			       u,
			       u_norm,
			       u_arctan,
			       drot1,
			       dtrans,
			       drot2);
    boost::python::list l;
    l.append(drot1);
    l.append(dtrans);
    l.append(drot2);
    return l;
}

void motion_model::compute_relative_transform(pyarr<double> &pose,
				pyarr<double> &u,
				float u_norm,
				float u_arctan,
				double &drot1,
				double &dtrans,
				double &drot2)
{
    drot1 = u_arctan - pose[ind(2)];
    dtrans = (double)u_norm;
    drot2 = u[ind(2)] - drot1;
}

boost::python::list motion_model::py_update_pose_with_sample(pyarr<double> pose,
							     pyarr<double> sample)
{
    vector<double> sample_vec;
    sample_vec.push_back(sample[ind(0)]);
    sample_vec.push_back(sample[ind(1)]);
    sample_vec.push_back(sample[ind(2)]);

    vector<double> new_pose = update_pose_with_sample(pose,
						      sample_vec);
    boost::python::list l;
    l.append(new_pose[0]);
    l.append(new_pose[1]);
    l.append(new_pose[2]);
    return l;
}

vector<double> motion_model::update_pose_with_sample(pyarr<double> &pose,
						    vector<double> &sample)
{
    vector<double> new_pose(3, 0);
    
    // cout << "old pose[0]" << pose[ind(0)] << endl;
    // cout << "old pose[1]" << pose[ind(1)] << endl;
    // cout << "old pose[2]" << pose[ind(2)] << endl;
    new_pose[0] = pose[ind(0)] + sample[1] * cos(pose[ind(2)] + sample[0]);
    new_pose[1] = pose[ind(1)] + sample[1] * sin(pose[ind(2)] + sample[0]);

    double before = pose[ind(2)] + sample[0] + sample[2];
    // cout << "before: " << before << endl;
    new_pose[2] = true_mod((pose[ind(2)] + sample[0] + sample[2]), 2*M_PI);

    // cout << "new pose[0]: " << new_pose[0] << endl;
    // cout << "new pose[1]: " << new_pose[1] << endl;
    // cout << "new pose[2]: " << new_pose[2] << endl;
    return new_pose;
}


pyarr<double> motion_model::update(pyarr<double> pose, 
				   pyarr<double> u,
				   float u_norm,
				   float u_arctan)
{
    double drot1, dtrans, drot2;
    
    compute_relative_transform(pose,
			       u,
			       u_norm,
			       u_arctan,
			       drot1,
			       dtrans,
			       drot2);
    
    double drot1_sq = pow(drot1, 2);
    double dtrans_sq = pow(dtrans, 2);
    double drot2_sq = pow(drot2, 2);

    double num = normal_generator();

    vector<double> sample;
    
    sample.push_back(drot1 + sqrt(alpha1 * drot1_sq + alpha2 * dtrans_sq) * normal_generator());
    sample.push_back(dtrans + sqrt(alpha3 * dtrans_sq + alpha4 * drot1_sq + alpha4 * drot2_sq) * normal_generator());
    sample.push_back(drot2 + sqrt(alpha1 * drot2_sq + alpha2 * dtrans_sq) * normal_generator());

    vector<long int> dims;
    dims.push_back(3);
    pyarr<double> new_pose(dims);

    vector<double> new_pose_vec = update_pose_with_sample(pose, sample);
    new_pose[ind(0)] = new_pose_vec[0];
    new_pose[ind(1)] = new_pose_vec[1];
    new_pose[ind(2)] = new_pose_vec[2];
    return new_pose;
}

BOOST_PYTHON_MODULE(libmotion_model)
{
    import_array();
    boost_common();
    
    class_<motion_model>("motion_model", init<float, float, float, float>())
	.def("update", &motion_model::update)
	.def("compute_relative_transform_float", &motion_model::compute_relative_transform_float)
	.def("py_update_pose_with_sample", &motion_model::py_update_pose_with_sample)
	;
}
