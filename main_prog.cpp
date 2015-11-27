#include "map.h"
#include "SensorData.h"
#include "particle_filter.h"
#include "matplotlibcpp.h"
#include <stdio.h>
#include <random>
#include <string>

#define PI 3.1415926
#define num_particles 50000
#define num_particles_exploitation 1000

float laser_variance = 0.1;

#pragma warning( push )
#pragma warning (disable:4101)

namespace plt = matplotlibcpp;

int main()
{
	//Preprocess data
	//Create exploitation bool
	bool start_exploitation = false;

	//Read map
	char* mapName = "map/wean.dat";
	map* map_obj = new map;
	map_obj->map_read(mapName);
	//map_obj->map_print();
	std::cout<<"Map Read."<<std::endl;
	//Read sensor data

	SensorData* laser_obj = new SensorData;
	std::string logName = "robotdata1";
	std::string logFile = "log/" + logName + ".log";
	const char* laserLog = logFile.c_str();
	laser_obj->read_data(laserLog);
	char imgID[8];

	//laser_obj->print_data();
	std::cout<<"Sensor Data got."<<std::endl;
	//Initialize particles
	std::vector< std::vector<float> >* map_occupancy = map_obj->get_map_grid();
	std::vector<particle*>* particles = new std::vector<particle*>;
	
	//Create three distrbutions for x y theta
	std::default_random_engine generator;

	std::uniform_int_distribution<int> distribution_x(0,(int)map_obj->get_size_x() - 1);
	std::uniform_int_distribution<int> distribution_y(0,(int)map_obj->get_size_y() - 1);
	std::uniform_real_distribution<float> distribution_theta(0, PI);

	int candidate_x = 0;
	int candidate_y = 0;
	float candidate_theta = 0.0;

	while(particles->size() < num_particles)
	{
		candidate_x = distribution_x(generator);
		candidate_y = distribution_y(generator);
		candidate_theta = distribution_theta(generator);
		if ((*map_occupancy)[candidate_x][candidate_y] == -1.0 || (*map_occupancy)[candidate_x][candidate_y] >= 0.3 )
			continue;
		else
		{
//			std::cout << "X Y Z equal " << candidate_x << " " << candidate_y << " " << candidate_theta << std::endl;
			particle* new_particle = new particle((float)candidate_x,(float)candidate_y, candidate_theta);
			particles->push_back(new_particle);
		}
	}
	std::cout<<"Particles Initialized."<<std::endl;

	//Preprocess odometry and sensor data
	std::vector<char>* ordering = laser_obj->get_ordering();
	std::vector<float>* cur_odom = new std::vector<float>;
	std::vector<int>* cur_laser;
	bool no_laser = false;
	int laser_iter = 0;
	int* seen_array = new int[num_particles];

	//Preprocess plot stuff
	std::vector<float> plot_x;
	std::vector<float> plot_y;
	
	std::vector<int> wall_x;
	std::vector<int> wall_y;

	
	for (int x = 0; x< map_obj->get_size_x(); x++)
		for (int y = 0; y< map_obj->get_size_y(); y++)
			if((*map_occupancy)[x][y] >= 0.2 && (*map_occupancy)[x][y] <= 1)
			{
				wall_x.push_back(x);
				wall_y.push_back(y);
			}



	for (int t = 1; t< ordering->size(); t++)
	{
		//plot stuff

		std::cout << "T = " << t << std::endl;
		//Calculate the odometry
		cur_odom->clear();
		std::cout<<"Cur odom Cleared"<<std::endl;
		for (int i = 0; i < 3; i++)
			cur_odom->push_back((*(laser_obj->get_odom_data()))[t][i] - (*(laser_obj->get_odom_data()))[t-1][i]);
		std::cout<<"New cur odom done"<<std::endl;

		if ((*ordering)[t] == 'O')
		{
			no_laser = true;

			//No need to change laser data
			std::cout<<"Entered Odometry step"<<std::endl;
			particles = particle_filter(map_obj, particles, cur_odom, cur_laser, &(*(laser_obj->get_laser_relative_pose()))[laser_iter], laser_variance, no_laser, map_obj->get_size_x(), map_obj->get_size_y(), seen_array, start_exploitation);
			no_laser = false;
			std::cout<<"Odometry step."<<std::endl;
		}
		else if((*ordering)[t] == 'L')
		{
			cur_laser = &(*(laser_obj->get_laser_scan()))[laser_iter];
			particles = particle_filter(map_obj, particles, cur_odom, cur_laser, &(*(laser_obj->get_laser_relative_pose()))[laser_iter], laser_variance, no_laser, map_obj->get_size_x(), map_obj->get_size_y(), seen_array, start_exploitation);
			laser_iter++;
			std::cout<<"Laser step."<<std::endl;
		}
		
		//Exploitation
		if (t == 200)
			start_exploitation = true;

		//Plotting
		if (t%5 == 0)
		{	
			plot_x.clear();
			plot_y.clear();
			if (start_exploitation)
			{
				for (int i=0; i<num_particles_exploitation; i++)
				{
					plot_x.push_back((*particles)[i]->get_x());
					plot_y.push_back((*particles)[i]->get_y());
				}
			}
			else
			{	
				for (int i=0; i<num_particles; i++)
				{
					plot_x.push_back((*particles)[i]->get_x());
					plot_y.push_back((*particles)[i]->get_y());
				}
			}
			sprintf( imgID, "%08d",t);
			const std::string fileName = "results/" + logName + "/" + imgID + ".png";
			plt::plot(wall_x, wall_y, "k*");
			plt::plot(plot_x, plot_y, "r.");
			plt::save(fileName);
			plt::close();

		}	
	}
	for (std::vector<particle*>::iterator iter_particles = particles->begin(); iter_particles != particles->end(); iter_particles++)
	{
		std::cout<<"About to free"<<std::endl;
		delete(*iter_particles);
		std::cout<<"Freed individual particles."<<std::endl;
	}
	
	particles->clear();
	std::cout<<"Cleared particles vector."<<std::endl;
	delete(particles);
	std::cout<<"Deleted particles vector."<<std::endl;
	delete(cur_odom);
	std::cout<<"Deleted odom vector."<<std::endl;
	
	return 0;
}

#pragma warning (pop)