#include "map.h"
#include <stdio.h>

void map::map_print()
{
	for (int i =0; i< size_x; i++)
	{
		for (int j = 0; j < size_y; j++)
		{
			printf("%f\t", map_grid[i][j]);
		}
		printf("\n");
	}

}

int map::map_read(char* mapName){
	FILE* fp;
	fp = fopen(mapName, "r");
	int x, y, count;
	float temp;
	char line[256];
	if(fp == NULL)
	{
		printf("Could not open file %s\n", mapName);
		return -1;
	}
	fprintf(stderr, "# Reading map: %s\n", mapName);
	while((fgets(line, 256, fp) != NULL)
		&& (strncmp("global_map[0]", line , 13) != 0)) {
	    if(strncmp(line, "robot_specifications->resolution", 32) == 0)
	      if(sscanf(&line[32], "%d", &(resolution)) != 0)
		printf("# Map resolution: %d cm\n", resolution);
	    if(strncmp(line, "robot_specifications->autoshifted_x", 35) == 0)
	      if(sscanf(&line[35], "%g", &(offset_x)) != 0) {
		printf("# Map offsetX: %g cm\n", offset_x);
	      }
	    if(strncmp(line, "robot_specifications->autoshifted_y", 35) == 0) {
	      if (sscanf(&line[35], "%g", &(offset_y)) != 0) {
		printf("# Map offsetY: %g cm\n", offset_y);
	      }
	    }
	  }
	if(sscanf(line,"global_map[0]: %d %d", &size_y, &size_x) != 2) 
	{
		fprintf(stderr, "ERROR: corrupted file %s\n", mapName);
		fclose(fp);
		return -1;
	}
	printf("# Map size: %d %d\n", size_x, size_y);

	min_x = size_x;
	max_x = 0;
	min_y = size_y;
	max_y = 0;
	count = 0;
	std::vector<float> map_y;
	for(x = 0; x < size_x; x++)
	{
		if (map_y.size() != 0)
			map_y.clear();
		for(y = 0; y < size_y; y++, count++) 
		{
			if(count % 10000 == 0)
				fprintf(stderr, "\r# Reading ... (%.2f%%)",
					count / (float)(size_x * size_y) * 100);
		  
		  	fscanf(fp,"%e", &temp);
		  	if(temp < 0.0)
				map_y.push_back(-1.0);
		  	else 
		  	{
				if(x < min_x)
				  	min_x = x;
				else if(x > max_x)
				  	max_x = x;
				if(y < min_y)
				  	min_y = y;
				else if(y > max_y)
				  	max_y = y;
				map_y.push_back(1 - temp);
			}
		}
		map_grid.push_back(map_y);
	}
	fprintf(stderr, "\r# Reading ... (%.2f%%)\n\n",
	  count / (float)(size_x * size_y) * 100);
	fclose(fp);
	return 0;

}