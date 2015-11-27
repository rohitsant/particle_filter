#include <memory>
#include "particle_filter.h"
#include "map.h"
#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include <random>
#include <algorithm>

#define PI 3.14159265
#define num_particles_exploitation 1000


float compute_weight(map* map_data, particle* current_particle, std::vector<int>* laser_observation_t, std::vector<float>* laser_relative_pose_t, float laser_variance, int size_x, int size_y)
{
	int count = 0;
	float cumul_weight = 0;
	float weight = 0;


	for (std::vector<int>::iterator iter_laser = laser_observation_t->begin(); iter_laser != laser_observation_t->end(); iter_laser++)
	{
		//Noise calculation
		//Create observation model Gaussian noise
		std::normal_distribution<double> distribution_r(0.0,0.5);
		std::normal_distribution<double> distribution_theta(0.0, 0.0035);
		std::default_random_engine generator;



		//Resolution = 10
		int x = (int) (10*current_particle->get_x() + (*iter_laser)*cos(distribution_theta(generator) + count*PI/180) + (*laser_relative_pose_t)[0] + distribution_r(generator))/10;
		int y = (int) (10*current_particle->get_y() + (*iter_laser)*sin(distribution_theta(generator) + count*PI/180) + (*laser_relative_pose_t)[1] + distribution_r(generator))/10;
		
		//Calculate weight. To do this, pick the right 
		if (x > 0 && y > 0 && x < size_x && y < size_y)
			weight = (*(map_data->get_map_grid()))[x][y]; //Remove the 1- if it turns out that the map fill-in is inverted
		else
			weight = 0; 
		
		if (weight < 0)
			weight = 0;

		std::vector <std::vector<float> >* map_grid = (map_data->get_map_grid());
		if (current_particle->get_x() > 0 && current_particle->get_y() > 0 && current_particle->get_x() < size_x && current_particle->get_x() < size_y)
		{	
			if ((*map_grid)[(int)current_particle->get_x()][(int)current_particle->get_y()] >= 0.3)
		 		weight = 0;
		}
		else 
			weight = 0;
		//std::cout<<"Weight = "<< weight << std::endl;
		cumul_weight = cumul_weight + pow(weight,2);
		count++;
	}
	return cumul_weight;
}

std::vector<particle*>* particle_filter(map* map_data, std::vector< particle* >* X_prev, std::vector<float>* odom_t, std::vector<int>* laser_observation_t, std::vector<float>* laser_relative_pose_t, float laser_variance, bool no_laser, int size_x, int size_y, int* seen_array, bool start_exploitation)
{
	//Initialize temporary particles vector 
	std::vector< particle* >* temp_particles = new std::vector <particle*>;
	std::vector<float> weights;

	//Extract number of particles
	int num_particles = X_prev->size();

	//Create motion model Gaussian noise - NOTE THAT THE XY Coordinates are in Map resolution
	std::normal_distribution<double> distribution_xy(0.0,0.25);
	std::normal_distribution<double> distribution_theta(0.0, 0.017);
	std::default_random_engine generator_noise;

	//Candidate distribution generation
	for (std::vector<particle*>::iterator iter_particles = X_prev->begin(); iter_particles != X_prev->end(); iter_particles++)
	{
		particle* new_particle = new particle;

		new_particle->set_x((*iter_particles)->get_x() + (*odom_t)[0]/10 + distribution_xy(generator_noise));
		new_particle->set_y((*iter_particles)->get_y() + (*odom_t)[1]/10 + distribution_xy(generator_noise));
		new_particle->set_theta((*iter_particles)->get_theta() + (*odom_t)[2] + distribution_theta(generator_noise));
		temp_particles->push_back(new_particle);

	}

	//Check if inertial preintegration case
	if (no_laser)
	{
		//Clear old particles
		int iter_count = 0;
		for (std::vector<particle*>::iterator iter_particles = X_prev->begin(); iter_particles != X_prev->end(); iter_particles++)
		{
			
			if (seen_array[iter_count] == 0)
				delete(*iter_particles);
			iter_count++;
		}
		X_prev->clear();
		

		//Set new particles as temp particles
		for (int i=0; i<num_particles; i++)
		{
			X_prev->push_back((*temp_particles)[i]);
		}

		temp_particles->clear();
		delete(temp_particles);
		return X_prev;
	}
	//Else there is a laser reading
	//Calculate weights

	
	for (std::vector<particle*>::iterator iter_particles = temp_particles->begin(); iter_particles != temp_particles->end(); iter_particles++)
	{
		weights.push_back(compute_weight(map_data,*iter_particles, laser_observation_t, laser_relative_pose_t, laser_variance, size_x, size_y));
	}

	//Resample
	std::default_random_engine generator;
	std::vector<float>::iterator weight_front = weights.begin();
	std::vector<float>::iterator weight_back = weights.end();
	std::discrete_distribution<int>* weight_dist = new std::discrete_distribution<int>(weight_front, weight_back);
	
	//Free the older particles
	
	int iter_count = 0;
	for (std::vector<particle*>::iterator iter_particles = X_prev->begin(); iter_particles != X_prev->end(); iter_particles++)
	{
		if (seen_array[iter_count] == 0)	
			delete(*iter_particles);
		iter_count++;
	}
	X_prev->clear();
	//Fill up the new particles in X_prev
	int random_pick = 0;
	std::vector<int> picked_particles;
	std::vector<int>::iterator it;

	if (start_exploitation)
		num_particles = num_particles_exploitation;

	std::cout << num_particles << std::endl;
	for (int i = 0; i< num_particles; i++)
	{
		random_pick = (*weight_dist)(generator);
		it = std::find(picked_particles.begin(), picked_particles.end(), random_pick);
		if (it != picked_particles.end())
			seen_array[i] = 1;
		else
		{
			picked_particles.push_back(random_pick);
			seen_array[i] = 0;
		}
		X_prev->push_back((*temp_particles)[random_pick]);
	}

	temp_particles->clear();
	delete(temp_particles);
	delete(weight_dist);

	return X_prev;

}

