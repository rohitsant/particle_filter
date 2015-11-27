#ifndef SENSORDATA_H
#define SENSORDATA_H

#include <stdio.h>
#include <string>
#include <vector>


class SensorData{
private:
	std::vector< std::vector<int> > laser_scan;
	std::vector< std::vector<float> > laser_relative_pose;
	std::vector<float> laser_relative_pose_stamps;

	std::vector<char> ordering;

	std::vector<float> odom_pose_stamps;
	std::vector<std::vector<float> > odom_data;
 
public:
	~SensorData();
	void read_data(char* fileName);
	void print_data();
	std::vector< std::vector<int> >* get_laser_scan(){return &laser_scan;};
	std::vector< std::vector<float> >* get_laser_relative_pose(){return &laser_relative_pose;};
	std::vector<float>* get_laser_relative_pose_stamps(){return &laser_relative_pose_stamps;};

	std::vector<std::vector<float> >* get_odom_data(){return &odom_data;};
	std::vector<float>* get_odom_pose_stamps(){return &odom_pose_stamps;};
	std::vector<char>* get_ordering(){return &ordering;};

};

#endif