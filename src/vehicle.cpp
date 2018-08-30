#include "vehicle.h"

Vehicle::Vehicle(){
	// Start vehicle in KL state in the middle lane stopped
	state 			= 0;
	lane  			= 1;
	ref_vel			= 0;
	max_ref_vel 	= 49.5;
	ref_vel_delta 	= 0.224; // 5 m/s^2 average acceleration
	Ts 				= 0.02;
	path_size		= 50;
	cp_inc			= 30;
	cur_front_car	= false;
	cur_rear_car		= false;
	left_front_car	= false;
	left_rear_car	= false;
	right_front_car	= false;
	right_rear_car	= false;
	
	// Vehicle IDs of closest surround cars will be saved here
	cur_front_id	= -1;
	cur_rear_id	= -1;
	left_front_id	= -1;
	left_rear_id	= -1;
	right_front_id= -1;
	right_rear_id	= -1;
}

Vehicle::Vehicle(int lane, double max_ref_vel){
	// Start vehicle in KL state in the middle lane stopped
	Vehicle();
	this->lane  			= lane;
	this->max_ref_vel 	= max_ref_vel;
}

Vehicle::~Vehicle(){}

void Vehicle::get_surrounding_vehicles(std::vector< std::vector<double> > sensor_fusion) {
	// Assume limits for max and min distances in [m] for obstacle cars in all lanes
	double cur_min_front_s		= 1000;
	double cur_max_rear_s		= 0;
	double left_min_front_s		= 1000;
	double left_max_rear_s	 	= 0;
	double right_min_front_s	= 1000;
	double right_max_rear_s		= 0;
	
	// Temporary variables to store Frenet coordinates of surrounding cars
	double obstacle_d	= 0;
	double obstacle_s	= 0;
	
	// Assume no immediate obstacles at start of check
	cur_front_car		= false;	
	cur_rear_car		= false;
	left_front_car		= false;
	left_rear_car		= false;
	right_front_car	= false;
	right_rear_car		= false;
	
	// Vehicle IDs of closest surround cars will be saved here
	cur_front_id	= -1;
	cur_rear_id		= -1;
	left_front_id	= -1;
	left_rear_id	= -1;
	right_front_id	= -1;
	right_rear_id	= -1;


	
	// Check for closest surrounding vehicles in all lanes
	for (int i = 0; i< sensor_fusion.size(); i++){
		d = sensor_fusion[i][6];
		// Check for front and rear vehicle in same lane
		if ((obstacle_d > 4*this->lane) && (obstacle_d < 4+4*this->lane)){
			obstacle_s = sensor_fusion[i][5];
			// Check if this is closest front car
			if((obstacle_s > this->s) && (obstacle_s < cur_min_front_s)){
				cur_min_front_s = obstacle_s;
				cur_front_id    = i;
				// Set flag for front car
				cur_front_car = true;
			}
			// Check if this is closest rear car
			else if((obstacle_s < this->s) && (obstacle_s > cur_max_rear_s)){
				cur_max_rear_s = obstacle_s;
				cur_rear_id 	= i;
				// Set flag for rear car
				cur_rear_car = true;				
			}
		}
		// Check for front and rear vehicle in left lane
		else if((obstacle_d > 0) && (obstacle_d < 4*this->lane)){
			obstacle_s = sensor_fusion[i][5];
			// Check if this is closest left front car
			if((obstacle_s > this->s) && (obstacle_s < left_min_front_s)){
				left_min_front_s = obstacle_s;
				left_front_id    = i;
				// Set flag for front car
				left_front_car = true;
			}
			// Check if this is closest left rear car
			else if((obstacle_s < this->s) && (obstacle_s > left_max_rear_s)){
				left_max_rear_s = obstacle_s;
				left_rear_id 	= i;
				// Set flag for rear car
				left_rear_car = true;				
			}
		}
		// Check for front and rear vehicle in right lane
		else if((obstacle_d > 4*this->lane+4) && (obstacle_d < 4*this->lane+8)){
			obstacle_s = sensor_fusion[i][5];
			// Check if this is closest right front car
			if((obstacle_s > this->s) && (obstacle_s < left_min_front_s)){
				left_min_front_s = obstacle_s;
				left_front_id    = i;
				// Set flag for front car
				left_front_car = true;
			}
			// Check if this is closest right rear car
			else if((obstacle_s < this->s) && (obstacle_s > left_max_rear_s)){
				left_max_rear_s = obstacle_s;
				left_rear_id 	= i;
				// Set flag for rear car
				left_rear_car = true;				
			}
		}
	}
	
}

void Vehicle::choose_next_state(std::vector< std::vector<double> > sensor_fusion){
	if(cur_front_car){
		// Check gap between front vehicle and ego vehicle
		if(sensor_fusion[cur_front_id][5] - s < cp_inc){
			state = 1; // Go to PCLC
		}
		else{
			state = 0; // Stay in KL
		}
	}
	else if(ref_vel < max_ref_vel){
		
		ref_vel += ref_vel_delta;
	}
	// Project the obstacle's Frenet position prev_path_size steps into the future
	//obstacle_s += (double)prev_path_size*Ts*obstacle_speed;

	//double vx = sensor_fusion[i][3];
	//double vy = sensor_fusion[i][4];
	//double obstacle_speed = sqrt(vx*vx + vy*vy);
	/*// FSM for changing lanes
	if(this->lane > 0){
		this->lane -= 1;
	}else if (this->lane < 2){
		this->lane +=1;
	}*/
}