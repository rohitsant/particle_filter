#ifndef SENSORDATA_H
#define SENSORDATA_H

#include <stdio.h>
#include <string>
#include <vector>


class SensorData{
private:
	std::vector< std::vector<int> > laser_scan;
	std::vector< std::vector<float> > robot_pose;
	std::vector< std::vector<float> > laser_pose;
	std::vector<char> ordering;

	std::vector<std::vector<float> > odom_data;
 
public:
	~SensorData();
	void read_data(char* fileName);
	void print_data();
	std::vector< std::vector<int> >* get_laser_scan(){return &laser_scan;};
	std::vector< std::vector<float> >* get_robot_pose(){return &robot_pose;};
	std::vector< std::vector<float> >* get_laser_pose(){return &laser_pose;};
	std::vector<float>* get_laser_pose_stamps(){return &laser_pose_stamps;};

	std::vector<std::vector<float> >* get_odom_data(){return &odom_data;};
	std::vector<float>* get_odom_pose_stamps(){return &odom_pose_stamps;};

};

#endif