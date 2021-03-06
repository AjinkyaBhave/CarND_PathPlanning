#ifndef VEHICLE_H
#define VEHICLE_H

#include <math.h>
#include <iostream>
#include <vector>

#define STATE_KL		0
#define STATE_PLCL	1
#define STATE_PLCR	2
#define STATE_LCL		3
#define STATE_LCR		4
#define STATE_LC		5

#define LEFT_LANE		0
#define CENTRE_LANE	1
#define RIGHT_LANE	2

#define ROAD_LENGTH 6945.554
// Conversion from mph to m/s
#define MPH_TO_MPS  0.447

using namespace std;

// Class to store ego vehicle-related state and parameters
class Vehicle{
	
	public:
		// Members
		// Start state of ego vehicle. KL=0, PLCL=1, PLCR=2, LCL=3, LCR=4
		int state;
		// Starting lane of ego vehicle. Lane closest to the centre line is 0. Middle lane is 1. Right lane is 2.
		int lane;
		// Width of each lane in [m]
		double lane_width;
		// Global coordinates in [m]
		double x;
		double y;
		// Frenet coordinates in [m]
		double s;
		double d;
		// Motion variables
		double yaw;		// [rad]
		double speed;	// [mph]
		// Reference speed of ego vehicle [mph]
		double ref_vel;
		// Maximum reference speed of ego vehicle [mph]
		double max_ref_vel;
		// Change in ref_vel in mph to achieve target average acceleration
		double ref_vel_delta;
		// Lateral offset from lane centre in [m]. Used to check when lane changing is complete.
		double lane_offset;
		// Sample time of simulator in seconds
		double Ts;
		// Length of current path in number of points 
		int path_size;
		// Distance between successive control points in Frenet coordinates in metres
		double cp_inc;

		// Indicates if surrounding car of this type is present
		bool cur_front_car;
		bool cur_rear_car;
		bool left_front_car;
		bool left_rear_car;
		bool right_front_car;
		bool right_rear_car;
		
		// Vehicle IDs of closest surround cars will be saved here
		int cur_front_id;
		int cur_rear_id;
		int left_front_id;
		int left_rear_id;
		int right_front_id;
		int right_rear_id;
		
		// Speed of obstacle front vehicle [mph]
		double cur_front_speed;
		// Minimum speed above which car should change lanes [mph]
		double min_change_speed;
		
		// Timer to stay in STATE_PLCL
		int PLCL_count;
		// Timer to stay in STATE_PLCR
		int PLCR_count;
		// Timer threshold to switch back to STATE_KL from STATE_PLCx
		int PLC_count_threshold;
		// Distance of ego car from lane centre to check end of STATE_LCx
		double lane_ctr_threshold; 
		// Timer to bias vehicle to stay in centre lane
		int KL_count;
		// Timer threshold to bias vehicle to stay in centre lane
		int KL_count_threshold;

		// Methods
		// Constructors
		Vehicle();
		Vehicle(int lane, double max_ref_vel);
		//Destructor
		virtual ~Vehicle();
		void get_surrounding_vehicles(vector<vector<double> > sensor_fusion, int prev_path_size, double end_path_s);
		void choose_next_state();
		void state_KL();
		void state_PLCL();
		void state_PLCR();
		void state_LC();
};
#endif
