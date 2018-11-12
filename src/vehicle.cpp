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
	lane_offset 	= 0.05;
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
	
	PLCL_count	= 0;
	PLCR_count	= 0;
	PLC_count_threshold = 3;
	KL_count = 0;
	KL_count_threshold = 100;
	
	min_change_speed = 20*MPH_TO_MPS;
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
	// Temporary variables to store speeds of surrounding cars
	double vx = 0;
	double vy = 0;
	
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
	for (int i = 0; i < sensor_fusion.size(); i++){
		obstacle_d = sensor_fusion[i][6];
		// Check for front and rear vehicle in same lane
		if ((obstacle_d > lane_width*lane) && (obstacle_d < lane_width*(lane+1))){
			obstacle_s = sensor_fusion[i][5];
			// Check if this is closest front car
			if((obstacle_s > s) && (obstacle_s < cur_min_front_s)){
				cur_min_front_s = obstacle_s;
				cur_front_id    = i;
			}
			// Check if this is closest rear car
			else if((obstacle_s < s) && (obstacle_s > cur_max_rear_s)){
				cur_max_rear_s = obstacle_s;
				cur_rear_id 	= i;
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
				}
				// Check if this is closest left rear car
				else if((obstacle_s < s) && (obstacle_s > left_max_rear_s)){
					left_max_rear_s = obstacle_s;
					left_rear_id 	= i;
				}
				continue;
			}	
		}
		// Check for front and rear vehicle in right lane
		if(lane != RIGHT_LANE){ 
			if((obstacle_d > lane_width*(lane+1)) && (obstacle_d < lane_width*(lane+2))){
				obstacle_s = sensor_fusion[i][5];
				// Check if this is closest right front car
				if((obstacle_s > this->s) && (obstacle_s < right_min_front_s)){
					right_min_front_s = obstacle_s;
					right_front_id    = i;
				}
				// Check if this is closest right rear car
				else if((obstacle_s < this->s) && (obstacle_s > right_max_rear_s)){
					right_max_rear_s = obstacle_s;
					right_rear_id 	= i;
				}
				continue;
			}
		}
	}
	
	// Gap between obstacle vehicle and ego vehicle is too small
	if((cur_front_id != -1) && (sensor_fusion[cur_front_id][5] - s) < cp_inc){
		// Set flag for front car
		cur_front_car = true;
		vx = sensor_fusion[cur_front_id][3];
		vy = sensor_fusion[cur_front_id][4];
		cur_front_speed = sqrt(vx*vx + vy*vy);
	}
	if((cur_rear_id != -1) && (s - sensor_fusion[cur_rear_id][5]) < 0.2*cp_inc){
		// Set flag for rear car
		cur_rear_car = true;
	}
	if((left_front_id != -1) && (sensor_fusion[left_front_id][5] - s) < cp_inc){
		// Set flag for front car
		left_front_car = true;
	}
	if((left_rear_id != -1) && (s - sensor_fusion[left_rear_id][5]) < 0.2*cp_inc){
		// Set flag for rear car
		left_rear_car = true;
	}
	if((right_front_id != -1) && (sensor_fusion[right_front_id][5] - s) < cp_inc){
		// Set flag for front car
		right_front_car = true;
	}
	
	if( (right_rear_id != -1) && (s - sensor_fusion[right_rear_id][5]) < 0.2*cp_inc){
		// Set flag for rear car
		right_rear_car = true;
	}
	
	printf("Speed: %f\n", speed);
	//printf("Traffic- LF: %d, LR: %d, CF: %d, CR: %d, RF: %d  RR: %d\n", left_front_car , left_rear_car, cur_front_car, cur_rear_car, right_front_car, right_rear_car);
	
}

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
		// Gap between front vehicle and ego vehicle is too small
		printf("front car \n");
		// Reduce current speed
		if(cur_front_speed < speed){
			printf("Front Speed: %f, Speed: %f \n", cur_front_speed, speed);
			ref_vel -= ref_vel_delta;
		}
		// Check current lane to decide whether to change left or right
		if(lane != LEFT_LANE && PLCL_count == 0){
			printf("STATE KL to PLCL\n");
			state = STATE_PLCL; 
		}
		else if (lane != RIGHT_LANE && PLCR_count == 0){
			printf("STATE KL to PLCR\n");
			state = STATE_PLCR;
		}
		else{
			// Set both PLCx state counts to zero and try changing lanes again
			PLCL_count = 0;
			PLCR_count = 0;
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
		if (lane != CENTRE_LANE){
			KL_count = KL_count + 1;
		}
		else{
			KL_count = 0;
		}
		if(KL_count > KL_count_threshold){
			if(lane != LEFT_LANE){
				printf("STATE KL to PLCL\n");
				state = STATE_PLCL; 
			}
			else if (lane != RIGHT_LANE){
				printf("STATE KL to PLCR\n");
				state = STATE_PLCR;
			}
		}
	}
}

void Vehicle::state_PLCL(std::vector< std::vector<double> > sensor_fusion){
	// Allow to change lanes if situation safe
	bool change_lane = true;
	// Car in front in left lane
	if(left_front_car){
		// Gap between front left vehicle and ego vehicle is too small
		printf("left front car \n");
		if(cur_front_speed < speed){
			// Reduce current speed
			ref_vel -= ref_vel_delta;
		}
		// Not possible to safely change lanes 
		change_lane = false;
		PLCL_count = PLCL_count + 1;
		if(PLCL_count > PLC_count_threshold){
			state = STATE_KL;
			printf("STATE PLCR to KL\n");
		}
		return;
	}
	if(left_rear_car){
		// Gap between rear left vehicle and ego vehicle is too small
		printf("left rear car \n");
		if(cur_front_speed < speed){
			// Reduce current speed
			ref_vel -= ref_vel_delta;
		}
		// Not possible to safely change lanes 
		change_lane = false;
		PLCL_count = PLCL_count + 1;
		if(PLCL_count > PLC_count_threshold){
			state = STATE_KL;
			printf("STATE PLCL to KL\n");
		}
		return;
	}
	
	if(change_lane && (speed > min_change_speed)){
		state = STATE_LC;
		printf("STATE PLCL to LC\n");
		lane = lane - 1;
		PLCL_count = 0;
	}
}

void Vehicle::state_PLCR(std::vector< std::vector<double> > sensor_fusion){
	// Allow to change lanes if situation safe
	bool change_lane = true;
	// Car in front in right lane
	if(right_front_car){
		// Gap between front right vehicle and ego vehicle is too small
		printf("right front car \n");
		if(cur_front_speed < speed){
			// Reduce current speed
			ref_vel -= ref_vel_delta;
		}
		// Not possible to safely change lanes 
		change_lane = false;
		PLCR_count = PLCR_count + 1;
		if(PLCR_count > PLC_count_threshold){
			state = STATE_KL;
			printf("STATE PLCR to KL\n");
		}
		return;
	}
	if(right_rear_car){
		// Gap between rear right vehicle and ego vehicle is too small
		printf("right rear car \n");
		if(cur_front_speed < speed){
			// Reduce current speed
			ref_vel -= ref_vel_delta;
		}
		// Not possible to safely change lanes 
		change_lane = false;
		PLCR_count = PLCR_count + 1;
		if(PLCR_count > PLC_count_threshold){
			state = STATE_KL;
			printf("STATE PLCR to KL\n");
		}
		return;
	}
	
	if(change_lane && (speed > min_change_speed)){
		state = STATE_LC;
		printf("STATE PLCR to LC\n");
		lane = lane + 1;
		PLCR_count = 0;
	}
}

void Vehicle::state_LC(){
	double lane_centre = lane_width/2.0 + lane_width*lane;
	//printf("d: %f, centre: %f\n", d, lane_centre);
	if(abs(d - lane_centre)  < lane_offset){
		state = STATE_KL;
		printf("STATE LC to KL\n");
	}	
}
