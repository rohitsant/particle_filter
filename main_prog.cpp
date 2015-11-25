#include "map.h"
#include "SensorData.h"
#include "particle_filter.h"
#include "matplotlibcpp.h"
#include <stdio.h>

#define PI 3.1415926

int num_particles = 1024;
float laser_variance = 0.1;


int main()
{
	char* mapName = "map/wean.dat";
	map* map_obj = new map;
	map_obj->map_read(mapName);
	//map_obj->map_print();
	SensorData* laser_obj = new SensorData;
	char* laserLog = "log/robotdata1.log";
	laser_obj->read_data(laserLog);
	//laser_obj->print_data();


	int x_delta = (int)map_obj->get_size_x()/sqrt(num_particles/5);
	int y_delta = (int)map_obj->get_size_y()/sqrt(num_particles/5);
	float theta_delta[5] = {0, PI/4, PI/2, 3*PI/4, PI};
	std::vector<particle*>* particles;
	
	for (int x = 0; x< map_obj->get_size_x(); x+=x_delta)
		for (int y = 0; y< map_obj->get_size_y(); y+=y_delta)
			for (int j = 0; j<5; j++)
			{
				particle* new_particle = new particle((float)x,(float)y, theta_delta[j]);
				particles->push_back(new_particle);
			}
	for (int t = 0; t< laser_obj->get_laser_scan()->size(); t++)
	{
	
		particles = particle_filter(map_obj, particles, laser_obj->get_odom_data(), laser_obj->get_laser_scan(), laser_obj->get_laser_pose(), laser_variance);
	
	}

}