#include "map.h"
#include "SensorData.h"
#include "particle_filter.h"
#include "matplotlibcpp.h"
#include <stdio.h>

#define PI 3.1415926

int num_particles = 5120;
float laser_variance = 0.1;

#pragma warning( push )
#pragma warning (disable:4101)

int main()
{
	//Preprocess data

	//Read map
	char* mapName = "map/wean.dat";
	map* map_obj = new map;
	map_obj->map_read(mapName);
	//map_obj->map_print();
	std::cout<<"Map Read."<<std::endl;
	//Read sensor data
	SensorData* laser_obj = new SensorData;
	char* laserLog = "log/robotdata1.log";
	laser_obj->read_data(laserLog);
	//laser_obj->print_data();
	std::cout<<"Sensor Data got."<<std::endl;
	//Initialize particles
	int x_delta = (int)map_obj->get_size_x()/sqrt(num_particles/5);
	int y_delta = (int)map_obj->get_size_y()/sqrt(num_particles/5);
	std::cout<<"Sizes got."<<std::endl;
	float theta_delta[5] = {0, PI/4, PI/2, 3*PI/4, PI};
	std::vector<particle*>* particles = new std::vector<particle*>;
	
	for (int x = 0; x< map_obj->get_size_x(); x+=x_delta)
		for (int y = 0; y< map_obj->get_size_y(); y+=y_delta)
			for (int j = 0; j<5; j++)
			{
				particle* new_particle = new particle((float)x,(float)y, theta_delta[j]);
				particles->push_back(new_particle);
			}
	std::cout<<"Particles Initialized."<<std::endl;
	//Preprocess odometry and sensor data
	std::vector<char>* ordering = laser_obj->get_ordering();
	std::vector<float>* cur_odom = new std::vector<float>;
	std::vector<int>* cur_laser;
	bool no_laser = false;
	int laser_iter = 0;

	for (int t = 1; t< ordering->size(); t++)
	{
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
			particles = particle_filter(map_obj, particles, cur_odom, cur_laser, &(*(laser_obj->get_laser_relative_pose()))[laser_iter], laser_variance, no_laser, map_obj->get_size_x(), map_obj->get_size_y());
			no_laser = false;
			std::cout<<"Odometry step."<<std::endl;
		}
		else if((*ordering)[t] == 'L')
		{
			laser_iter++;
			cur_laser = &(*(laser_obj->get_laser_scan()))[laser_iter];
			particles = particle_filter(map_obj, particles, cur_odom, cur_laser, &(*(laser_obj->get_laser_relative_pose()))[laser_iter], laser_variance, no_laser, map_obj->get_size_x(), map_obj->get_size_y());
			std::cout<<"Laser step."<<std::endl;
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