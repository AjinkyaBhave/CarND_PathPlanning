#include "vehicle.h"
#include <math.h>
#include <iostream>

Vehicle::Vehicle(){
	// Start vehicle in KL state in the centre lane with zero speed
	state 			= STATE_KL;
	lane  			= CENTRE_LANE;
	lane_width		= 4.0;
	ref_vel			= 0.0;
	max_ref_vel 	= 49.5;
	ref_vel_delta 	= 0.224; // 5 m/s^2 average acceleration
	lane_offset 	= 0.1;
	Ts 				= 0.02;
	path_size		= 50;
	cp_inc			= 30;
	cur_front_car	= false;
	cur_rear_car	= false;
	left_front_car	= false;
	left_rear_car	= false;
	right_front_car= false;
	right_rear_car	= false;
	
	// Vehicle IDs of closest surround cars will be saved here
	cur_front_id	= -1;
	cur_rear_id		= -1;
	left_front_id	= -1;
	left_rear_id	= -1;
	right_front_id	= -1;
	right_rear_id	= -1;
	
	// Timer to stay in STATE_PLCx
	PLC_count	= 0;
	// Timer threshold to switch back to STATE_KL from STATE_PLCx
	PLC_count_threshold = 3;
}

Vehicle::Vehicle(int lane, double max_ref_vel){
	// Start vehicle in KL state in the middle lane stopped
	Vehicle();
	this->lane = lane;
	this->max_ref_vel = max_ref_vel;
}

Vehicle::~Vehicle(){}

void Vehicle::get_surrounding_vehicles(std::vector< std::vector<double> > sensor_fusion) {
	// Assume limits for max and min distances in [m] for obstacle cars in all lanes
	double cur_min_front_s		= ROAD_LENGTH;
	double cur_max_rear_s		= -1;
	double left_min_front_s		= ROAD_LENGTH;
	double left_max_rear_s	 	= -1;
	double right_min_front_s	= ROAD_LENGTH;
	double right_max_rear_s		= -1;
	
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
		obstacle_d = sensor_fusion[i][6];
		// Check for front and rear vehicle in same lane
		if ((obstacle_d > lane_width*lane) && (obstacle_d < lane_width*(lane+1))){
			obstacle_s = sensor_fusion[i][5];
			// Check if this is closest front car
			if((obstacle_s > s) && (obstacle_s < cur_min_front_s)){
				cur_min_front_s = obstacle_s;
				cur_front_id    = i;
				// Set flag for front car
				cur_front_car = true;
				//printf("ID: %d, obs s: %f, s: %f, obs_d: %f, d: %f\n", cur_front_id, obstacle_s, s, obstacle_d, d);
			}
			// Check if this is closest rear car
			else if((obstacle_s < s) && (obstacle_s > cur_max_rear_s)){
				cur_max_rear_s = obstacle_s;
				cur_rear_id 	= i;
				// Set flag for rear car
				cur_rear_car = true;
			}
			continue;
		}
		// Check for front and rear vehicle in left lane
		if(lane != LEFT_LANE) {
			if((obstacle_d > lane_width*(lane-1)) && (obstacle_d < lane_width*lane)){
				obstacle_s = sensor_fusion[i][5];
				// Check if this is closest left front car
				if((obstacle_s > s) && (obstacle_s < left_min_front_s)){
					left_min_front_s = obstacle_s;
					left_front_id    = i;
					// Set flag for front car
					left_front_car = true;
				}
				// Check if this is closest left rear car
				else if((obstacle_s < s) && (obstacle_s > left_max_rear_s)){
					left_max_rear_s = obstacle_s;
					left_rear_id 	= i;
					// Set flag for rear car
					left_rear_car = true;
				}
				continue;
			}	
		}
		// Check for front and rear vehicle in right lane
		if(lane != RIGHT_LANE){ 
			if((obstacle_d > lane_width*(lane+1)) && (obstacle_d < lane_width*(lane+2))){
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
				continue;
			}
		}
	}
}

/*std::vector<int> Vehicle::successor_states() {
	
	// Temporary array to store successor states
	std::vector<int> states;
	
	// FSM to decide next possible states from current state
	if(state == STATE_KL){
		states.push_back(STATE_KL);
		states.push_back(STATE_PLCL);
		states.push_back(STATE_PLCR);
	}else if(state == STATE_PLCL){
		if(lane != LEFT_LANE) {
			states.push_back(STATE_PLCL);
			states.push_back(STATE_LCL);
		}
	}else if(state == STATE_PLCR){
		if(lane != RIGHT_LANE){
			states.push_back(STATE_PLCR);
			states.push_back(STATE_LCR);
		}
	}else if(state == STATE_LCL){
		if(fabs(car.d - 4*lane+2) < lat_offset){
			states.push_back(STATE_KL);
		}
	}
	//If state is "LCL" or "LCR", then just return "KL"
	return states;
}*/

void Vehicle::choose_next_state(std::vector< std::vector<double> > sensor_fusion)
{
	switch(state){
		case STATE_KL	: state_KL(sensor_fusion)	; break;
		case STATE_PLCL: state_PLCL(sensor_fusion); break;
		case STATE_PLCR: state_PLCR(sensor_fusion); break;
		case STATE_LC	: state_LC()					; break;
		default			: state = STATE_KL;
	}
	
	// Project the obstacle's Frenet position prev_path_size steps into the future
	//obstacle_s += (double)prev_path_size*Ts*obstacle_speed;
}

void Vehicle::state_KL(std::vector< std::vector<double> > sensor_fusion){
	if(cur_front_car){
		printf("front car \n");
		// Gap between front vehicle and ego vehicle is too small
		if((sensor_fusion[cur_front_id][5] - s) < cp_inc){
			printf("reduce speed\n");
			// Reduce current speed
			ref_vel -= ref_vel_delta;
			// Check current lane to decide whether to change left or right
			if(lane != LEFT_LANE){
				printf("new state: PLCL\n");
				state = STATE_PLCL; 
				PLC_count = 0;
			}
			else{
				printf("new state: PLCR\n");
				state = STATE_PLCR;
				PLC_count = 0;
			}
		}
		// Gap between front vehicle and ego vehicle is large enough to accelerate
		else{
			//double vx = sensor_fusion[cur_front_id][3];
			//double vy = sensor_fusion[cur_front_id][4];
			//double front_speed = sqrt(vx*vx + vy*vy);
			if(ref_vel < max_ref_vel){
				ref_vel += ref_vel_delta;
			}
		}
	}
	// No obstacle in front. Try to go at speed limit.
	else if(ref_vel < max_ref_vel){
			ref_vel += ref_vel_delta;
	}
}
void Vehicle::state_PLCL(std::vector< std::vector<double> > sensor_fusion){
	// Allow to change lanes if situation safe
	bool change_lane = true;
	// Car in front in left lane
	if(left_front_car){
		printf("left front car \n");
		// Gap between front left vehicle and ego vehicle is too small
		if((sensor_fusion[left_front_id][5] - s) < cp_inc){
			printf("reduce speed\n");
			// Reduce current speed
			ref_vel -= ref_vel_delta;
			// Not possible to safely change lanes 
			change_lane = false;
			PLC_count = PLC_count + 1;
			if(PLC_count > PLC_count_threshold){
				state = STATE_KL;
				PLC_count = 0;
			}
			return;
		}
	}
	if(left_rear_car){
		printf("left rear car \n");
		// Gap between rear left vehicle and ego vehicle is too small
		if((s - sensor_fusion[left_rear_id][5]) < cp_inc){
			printf("reduce speed\n");
			// Reduce current speed
			ref_vel -= ref_vel_delta;
			// Not possible to safely change lanes 
			change_lane = false;
			PLC_count = PLC_count + 1;
			if(PLC_count > PLC_count_threshold){
				state = STATE_KL;
				PLC_count = 0;
			}
			return;
		}
	}
	
	if(change_lane){
		state = STATE_LC;
		lane = lane - 1;
		PLC_count = 0;
	}
}

void Vehicle::state_PLCR(std::vector< std::vector<double> > sensor_fusion){
	// Allow to change lanes if situation safe
	bool change_lane = true;
	// Car in front in right lane
	if(right_front_car){
		printf("right front car \n");
		// Gap between front right vehicle and ego vehicle is too small
		if((sensor_fusion[right_front_id][5] - s) < cp_inc){
			printf("reduce speed\n");
			// Reduce current speed
			ref_vel -= ref_vel_delta;
			// Not possible to safely change lanes 
			change_lane = false;
			PLC_count = PLC_count + 1;
			if(PLC_count > PLC_count_threshold){
				state = STATE_KL;
				PLC_count = 0;
			}
			return;
		}
	}
	if(right_rear_car){
		printf("right rear car \n");
		// Gap between rear right vehicle and ego vehicle is too small
		if((s - sensor_fusion[lright_rear_id][5]) < cp_inc){
			printf("reduce speed\n");
			// Reduce current speed
			ref_vel -= ref_vel_delta;
			// Not possible to safely change lanes 
			change_lane = false;
			PLC_count = PLC_count + 1;
			if(PLC_count > PLC_count_threshold){
				state = STATE_KL;
				PLC_count = 0;
			}
			return;
		}
	}
	
	if(change_lane){
		state = STATE_LC;
		lane = lane + 1;
		PLC_count = 0;
	}
}

void Vehicle::state_LC(){
	double lane_centre = lane_width/2.0 + lane_width*lane;
	if(abs(d - lane_centre)  < lane_ctr_threshold){
		state = STATE_KL;
	}	
}
