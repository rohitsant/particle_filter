#include "particle_filter.h"
#include "map.h"
#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include <random>

#define PI 3.14159265



float compute_weight(map* map_data, particle* current_particle, std::vector<int>* laser_observation_t, std::vector<float>* laser_relative_pose_t, float laser_variance, int size_x, int size_y)
{
	int count = 0;
	float cumul_weight = 0;
	float weight = 0;
	for (std::vector<int>::iterator iter_laser = laser_observation_t->begin(); iter_laser != laser_observation_t->end(); iter_laser++)
	{
		
		int x = (int) (current_particle->get_x() + (*iter_laser)*cos(count*PI/180) + (*laser_relative_pose_t)[0]);
		int y = (int) (current_particle->get_y() + (*iter_laser)*sin(count*PI/180) + (*laser_relative_pose_t)[1]);
		

		if (x > 0 && y > 0 && x < size_x && y < size_y)
			weight = (*(map_data->get_map_grid()))[x][y]/laser_variance; //Remove the 1- if it turns out that the map fill-in is inverted
		else
			weight = 0; 
		
		//std::cout<<"Weight = "<< weight << std::endl;
		cumul_weight = cumul_weight + pow(weight,2);
		count++;
	}
	return cumul_weight;
}

std::vector<particle*>* particle_filter(map* map_data, std::vector< particle* >* X_prev, std::vector<float>* odom_t, std::vector<int>* laser_observation_t, std::vector<float>* laser_relative_pose_t, float laser_variance, bool no_laser, int size_x, int size_y)
{
	std::vector< particle* >* temp_particles = new std::vector <particle*>;
	std::vector<float> weights;

	int num_particles = X_prev->size();
	
	int debug_count = 0;
	//Candidate distribution generation
	for (std::vector<particle*>::iterator iter_particles = X_prev->begin(); iter_particles != X_prev->end(); iter_particles++)
	{
		particle* new_particle = new particle;

		new_particle->set_x((*iter_particles)->get_x() + (*odom_t)[0]);
		new_particle->set_y((*iter_particles)->get_y() + (*odom_t)[1]);
		new_particle->set_theta((*iter_particles)->get_theta() + (*odom_t)[2]);
		temp_particles->push_back(new_particle);
		
		debug_count++;
		std::cout<<"New particle count "<< debug_count << std::endl;

	}

	//Check if inertial preintegration case
	if (no_laser)
	{
		//Clear old particles
		int debug_count = 0;
		for (std::vector<particle*>::iterator iter_particles = X_prev->begin(); iter_particles != X_prev->end(); iter_particles++)
		{
			
			std::cout<<"No error here pls 1: "<<debug_count << std::endl;
			debug_count++;
			delete(*iter_particles);
		}
		std::cout<<"No error here pls 2"<<std::endl;
		X_prev->clear();
		
		//Set new particles as temp particles
		for (int i=0; i<num_particles; i++)
		{
			X_prev->push_back((*temp_particles)[i]);
		}
		std::cout<<"No error here pls 3"<<std::endl;
		temp_particles->clear();
		delete(temp_particles);
		return X_prev;
	}
	//Else there is a laser reading
	//Calculate weights
	weights.clear();
	
	std::cout<<"Weights cleared."<<std::endl;
	
	for (std::vector<particle*>::iterator iter_particles = X_prev->begin(); iter_particles != X_prev->end(); iter_particles++)
	{
		weights.push_back(compute_weight(map_data,*iter_particles, laser_observation_t, laser_relative_pose_t, laser_variance, size_x, size_y));
		//std::cout<<"Weight pushed back."<<std::endl;
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
	int random_pick = 0;
	for (int i=0; i<num_particles; i++)
	{
		random_pick = (*weight_dist)(generator);
		std::cout << "Random choice = " << random_pick << std::endl;
		X_prev->push_back((*temp_particles)[random_pick]);
	}
	temp_particles->clear();
	delete(temp_particles);
	delete(weight_dist);
	std::cout << "Size = " << X_prev->size() << std::endl;
	return X_prev;

}
