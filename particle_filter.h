#ifndef PARTICLE_H
#define PARTICLE_H

#include <iostream>
#include <string>
#include <vector>
#include "map.h"

class particle{
private:
	float x;
	float y;
	float theta;
public:
	particle(){x= -1; y = -1; theta = -1;};
	particle(float new_x, float new_y, float new_theta){x = new_x; y = new_y; theta = new_theta;};
	~particle(){};
	float get_x(){ return x;};
	void set_x(float new_x){x = new_x;};
	float get_y(){ return y;};
	void set_y(float new_y){y = new_y;};
	float get_theta(){ return theta;};
	void set_theta(float new_theta){theta = new_theta;};

};

std::vector<particle*>* particle_filter(map*, std::vector< particle* >* , std::vector<float>* , std::vector<int>* , std::vector<float>* , float, bool, int, int, int*, bool );

#endif