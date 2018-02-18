#ifndef OBSTACLES_H
#define OBSTACLES_H

struct Obstacle {
	int id;
	double x;
	double y;
	double v;
	double s;
	double d;
	double distance; //distance in frenet coordinates (s)
	int lane;

};


#endif