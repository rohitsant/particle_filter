#include "SensorData.h"
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>

void SensorData::read_data(char* fileName)
{
	std::ifstream logFile;
	char data_type;

	logFile.open(fileName, std::ios::in);
	if(!logFile){
		std::cout << "Can't open the log file"<< logFile << std::endl;
		exit(1);
	}
	float temp_value;
	std::vector<float> odom_pose_vector;
	std::vector<int> scan_line;
	std::vector<float> robot_pose_vector;
	std::vector<float> laser_pose_vector;
	

	while(logFile >> data_type)
	{

		if(data_type == 'O')
		{
			ordering.push_back(data_type);
			odom_pose_vector.clear();

			for (int i = 0; i<3; i++)
			{
				
				logFile >> temp_value;
				odom_pose_vector.push_back(temp_value);
			}
			odom_data.push_back(odom_pose_vector);
			logFile >> temp_value;
			odom_pose_stamps.push_back(temp_value);
		}
		else if(data_type == 'L')
		{
			ordering.push_back(data_type);
			scan_line.clear();
			robot_pose_vector.clear();
			laser_pose_vector.clear();
			
			for (int i = 0; i<3; i++)
			{
				
				logFile >> temp_value;
				robot_pose_vector.push_back(temp_value);
			}
			robot_pose.push_back(robot_pose_vector);
			for (int i = 0; i<3; i++)
			{
				
				logFile >> temp_value;
				laser_pose_vector.push_back(temp_value);
			}
			laser_pose.push_back(laser_pose_vector);
			
			float cur_value;
			while(logFile.peek() != '\n')
			{	
				logFile >> cur_value;
				scan_line.push_back((int)cur_value);
			}
			laser_pose_stamps.push_back(cur_value);
			scan_line.pop_back();
			laser_scan.push_back(scan_line);
			logFile.ignore(2, '\n');
		}
		
	}
}

void SensorData::print_data()
{

	for (std::vector< std::vector<int> >::iterator iter_out = laser_scan.begin(); iter_out != laser_scan.end(); iter_out++)
	{
		for (std::vector<int>::iterator iter_in = (*iter_out).begin(); iter_in != (*iter_out).end(); iter_in++)
		{
			printf("%d\t", *iter_in);
		}
		printf("\n");
	}
	for (std::vector< std::vector<float> >::iterator iter_out = odom_data.begin(); iter_out != odom_data.end(); iter_out++)
	{
		for (std::vector<float>::iterator iter_in = (*iter_out).begin(); iter_in != (*iter_out).end(); iter_in++)
		{
			printf("%f\t", *iter_in);
		}
		printf("\n");
	}


}