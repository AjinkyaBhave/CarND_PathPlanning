#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <vector>
#include <fstream>
#include <math.h>
#include <iostream>
#include "spline.h"
#include "json.hpp"

using namespace std;

class Trajectory_Generator{
	
	public:
	// Members
	// Load up map values for waypoints x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

	// The max s value before wrapping around the track back to 0
	double max_s;
	// Conversion from mph to m/s
	double mph_to_mps;
	
	// Methods
	// Constructor
	Trajectory_Generator();
	// Destructor
	virtual ~Trajectory_Generator();
	
	// For converting back and forth between radians and degrees.
	double pi() { return M_PI; }
	double deg2rad(double x) { return x * pi() / 180; }
	double rad2deg(double x) { return x * 180 / pi(); }
	
	double distance(double x1, double y1, double x2, double y2);
	int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);
	int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
	vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
	vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);
	void generate_trajectory(Vehicle car, vector<double>& next_x_vals, vector<double>& next_y_vals, vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s);
};
#endif