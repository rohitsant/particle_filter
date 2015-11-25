#include "particle_filter.h"
#include "map.h"
#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include <random>

#define PI 3.14159265



float compute_error(map* map_data, particle* current_particle, std::vector<int>* laser_observation_t, std::vector<float>* laser_relative_pose_t, float laser_variance)
{
	std::vector<float>* laser_map_frame;
	int count = 0;
	float cumul_err = 0;
	for (std::vector<int>::iterator iter_laser = laser_observation_t->begin(); iter_laser != laser_observation_t->end(); iter_laser++)
	{
		
		int x = (int) (current_particle->get_x() + (*iter_laser)*cos(count*PI/180) + (*laser_relative_pose_t)[0]);
		int y = (int) (current_particle->get_y() + (*iter_laser)*sin(count*PI/180) + (*laser_relative_pose_t)[1]);

		float err = (1-(*(map_data->get_map_grid()))[x][y])/laser_variance; //Remove the 1- if it turns out that the map fill-in is inverted
		cumul_err = cumul_err + pow(err,2);
		count++;
	}
	return cumul_err;
}

std::vector<particle*>* particle_filter(map* map_data, std::vector< particle* >* X_prev, std::vector<float>* odom_t, std::vector<int>* laser_observation_t, std::vector<float>* laser_relative_pose_t, float laser_variance)
{
	std::vector< particle* > temp_particles;
	std::vector<float> weights;

	int num_particles = X_prev->size();
	
	//Candidate distribution generation
	for (std::vector<particle*>::iterator iter_particles = X_prev->begin(); iter_particles != X_prev->end(); iter_particles++)
	{
		particle* new_particle = new particle;

		new_particle->set_x((*iter_particles)->get_x() + (*odom_t)[0]);
		new_particle->set_y((*iter_particles)->get_y() + (*odom_t)[1]);
		new_particle->set_theta((*iter_particles)->get_theta() + (*odom_t)[2]);
		temp_particles.push_back(new_particle);
	}

	//Calculate weights
	weights.clear();
	for (std::vector<particle*>::iterator iter_particles = X_prev->begin(); iter_particles != X_prev->end(); iter_particles++)
	{
		weights.push_back(compute_error(map_data,*iter_particles, laser_observation_t, laser_relative_pose_t, laser_variance));
		
	}

	//Resample
	std::default_random_engine generator;
	std::vector<float>::iterator weight_front = weights.begin();
	std::vector<float>::iterator weight_back = weights.end();
	std::discrete_distribution<int>* weight_dist = new std::discrete_distribution<int>(weight_front, weight_back);
	
	//Free the older particles
	for (std::vector<particle*>::iterator iter_particles = X_prev->begin(); iter_particles != X_prev->end(); iter_particles++)
	{
		delete(*iter_particles);
	}
	X_prev->clear();

	//Fill up the new particles in X_prev
	for (int i=0; i<num_particles; i++)
	{
		
		X_prev->push_back(temp_particles[(*weight_dist)(generator)]);
	}
	temp_particles.clear();
	delete(weight_dist);

	return X_prev;

}
