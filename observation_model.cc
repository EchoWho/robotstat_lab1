#include <boost_common.h>
#include <pyarr.h>
#include <cmath>

#include "motion_model.h"
#include "map_object.h"
using std::cout;
using std::endl;

class observation_model{

public:
    void init(float c_hit, float c_rand, float c_short, float c_max, 
	      float max_rng, float sigma,
	      map_object mo, motion_model mm)
    {
	this->c_hit = c_hit;
	this->c_rand = c_rand;
	this->c_short = c_short;
	this->c_max = c_max;

	this->max_rng = max_rng;
	this->sigma = sigma;
	   
	this->obs_map_object = mo;
	this->motion_model_object = mm;

	// alpha = (pow(10, 5) - 1) / max_rng;
	alpha = 0.0025;
	norm_const = 1.0 / (sigma * sqrt(2 * M_PI));
	sigma2 = pow(sigma, 2);
	assert(len(obs_map_object.coord_idx_lookup) > 0);
    }

    observation_model(float c_hit, float c_rand, float c_short, float c_max, 
		      float max_rng, float sigma,
		      map_object mo, motion_model mm)
    {
	init(c_hit, c_rand, c_short, c_max, 
	      max_rng, sigma,
	     mo, mm);
    }

    observation_model(map_object mo, motion_model mm)
    {
	init(4.0, 1, 0.1, 0.5, 1200.0, 50, mo, mm);
    }

    bool is_hit(vector<double> &pose)
    {
	int coord0;
	int coord1;
	obs_map_object.get_pose_coord(pose, coord0, coord1);
	return obs_map_object.is_hit(coord0, coord1);
    }

    double get_log_p_z_given_pose_u(double z, vector<double> &pose)
    {
	assert(pose.size() == 3);
	double z_expected = obs_map_object.get_z_expected(pose);
        double p_hit =   norm_const * std::exp(-std::pow(z - z_expected, 2) / (2 * sigma2));

	double p_z_given_x = 1;

	if (z > max_rng && z_expected > max_rng && std::abs(z - z_expected) < 500)
	{
	    p_z_given_x += c_max;
	}



	double p_short = exp(- z * alpha);

	p_z_given_x += c_hit * p_hit + c_short * p_short;

	// cout << "z: " << z << endl;
	// cout << "z_expected: " << z_expected << endl;
	// cout << "p_z_given_x " << p_z_given_x << endl;
	// double p_z_given_x = c_hit * p_hit + c_rand * 1.0;
	return std::log(p_z_given_x);
    }

    pyarr<double> update_particle_weights(pyarr<double> poses,
					  pyarr<double> laser_pose_offset,
					  pyarr<double> offsets,
					  pyarr<double> laser,
					  float faux_last_odom_theta)
    {
	vector<long int> dims;
	dims.push_back(poses.dims[0]);
	pyarr<double> weights(dims);
	weights.zero_data();
	vector<long int> pose_dims;
	pose_dims.push_back(3);
	    
	double offset_norm = offsets[ind(0)];
	double offset_arctan = offsets[ind(1)];


	// cout <<"offset norm: " << offset_norm << endl;
	// cout << " offset arctan: " << offset_arctan << endl;

	// cout << "dims[0]" << dims[0] << endl;

//	#pragma omp parallel for
	for(size_t i =0; i < dims[0]; i++)
	{

	    pyarr<double> pose(pose_dims);
	    pose[ind(0)] = poses[ind(i, 0)];
	    pose[ind(1)] = poses[ind(i, 1)];
	    pose[ind(2)] = poses[ind(i, 2)];

	    weights[ind(i)] = get_weight(pose,
					 laser_pose_offset,
					 offset_norm,
					 offset_arctan,
					 faux_last_odom_theta,
					 laser);
	}
	return weights;
    }

    double get_weight(pyarr<double> pose,
		      pyarr<double> laser_pose_offset,
		      double offset_norm,
		      double offset_arctan,
		      double faux_last_odom_theta,
		      pyarr<double> laser)
    {
	double drot1, dtrans, drot2;
	
	assert(len(obs_map_object.coord_idx_lookup) > 0);
	assert(laser.dims[0] == 180);

	motion_model_object.compute_relative_transform(faux_last_odom_theta,
						       laser_pose_offset,
						       offset_norm,
						       offset_arctan,
						       drot1,
						       dtrans,
						       drot2);
	
	vector<double> sample;
	sample.push_back(drot1);
	sample.push_back(dtrans);
	sample.push_back(drot2);

	vector<double> pose_vec;
	pose_vec.push_back(pose[ind(0)]);
	pose_vec.push_back(pose[ind(1)]);
	pose_vec.push_back(pose[ind(2)]);

	vector<double> new_pose = motion_model_object.update_pose_with_sample(pose, sample);
	new_pose[2] -= M_PI / 2.0;

	if (is_hit(new_pose) || is_hit(pose_vec))
	{
	    return 0;
	}

	double delt_theta = M_PI / 180.0;

	double log_weight_sum = 0;
	
	int sample_skip = 3;

	// ofstream writer("temp.txt");


	for(size_t l_idx = 0; l_idx < laser.dims[0]; l_idx++)
	{

	    if (l_idx % sample_skip == 0)
	    {
		double z = laser[ind(l_idx)];
		
		double logpz = get_log_p_z_given_pose_u(z, new_pose);
		log_weight_sum += logpz;
		// writer << logpz<< "\n";
	    }
	    

	    new_pose[2] = true_mod(new_pose[2] + delt_theta,
				   2 * M_PI);
	}

	double weight = std::exp(log_weight_sum * 0.4);
	
	// cout << "weight: " << weight << endl;
	// weight = pow(weight, 1.0 / 2.0);
	return weight;
    }
    
    const map_object & get_obs_map_object() {return obs_map_object;}
    void check_lookup_size();

    double c_hit, c_rand, c_short, c_max, max_rng, alpha;
    double hit_thresh;
    double resolution;
    double norm_const;
    double sigma, sigma2;
    double A;

    map_object obs_map_object;
    motion_model motion_model_object;
};

void observation_model::check_lookup_size()
{
    cout << "check fake init: " << obs_map_object.fake_init << endl;
    assert(len(obs_map_object.coord_idx_lookup) > 0);
}

BOOST_PYTHON_MODULE(libobservation_model)
{
    import_array();
    boost_common();
    boost_map_object();
    
    class_<observation_model>("observation_model", init<map_object, motion_model>())

	.def(init<float, float, float, float, float, float, map_object, motion_model>())
	.def("get_weight", &observation_model::get_weight)
	.def_readwrite("c_hit", &observation_model::c_hit)
	.def("update_particle_weights", &observation_model::update_particle_weights)
	.def("_check_lookup_size", &observation_model::check_lookup_size)
	;
}

