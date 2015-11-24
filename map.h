#ifndef MAP_H
#define MAP_H


#include <stdio.h>
#include <vector>
#include <string>

class map
{
private: 
	int size_x = 0;
	int size_y = 0;
	int min_x = 0;
	int max_x = 0;
	int min_y = 0;
	int max_y = 0;
	float offset_x = 0;
	float offset_y = 0;
	std::vector< std::vector<float> > map_grid; 
	int resolution = 0;

public:
	~map();
	void map_print();
	int map_read(char* mapName);

	std::vector< std::vector<float> >* get_map_grid(){return &map_grid;};
	int get_size_x(){return size_x;};
	int get_size_y(){return size_y;};

};

#endif