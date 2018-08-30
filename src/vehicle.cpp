#include "vehicle.h"

Vehicle::Vehicle(){
	// Start vehicle in KL state in the middle lane stopped
	state 			= 0;
	lane  			= 1;
	ref_vel			= 0;
	max_ref_vel 	= 49.5;
	ref_vel_dec 	= 0.224;
	Ts 				= 0.02;
	path_size		= 50;
	obstacle_close	= false;
}

Vehicle::Vehicle(int lane, double max_ref_vel){
	// Start vehicle in KL state in the middle lane stopped
	Vehicle();
	this->lane  			= lane;
	this->max_ref_vel 	= max_ref_vel;
}

Vehicle::~Vehicle(){}

bool Vehicle::get_surrounding_vehicles(vector< vector<double> > sensor_fusion ) {
	// Assume limits for max and min distances in [m] for obstacle cars in all lanes
	double cur_min_front_s		= 1000;
	double cur_max_rear_s		= 0;
	double left_min_front_s		= 1000;
	double left_max_rear_s	 	= 0;
	double right_min_front_s	= 1000;
	double right_max_rear_s		= 0;
	
	// Assume no immediate obstacles at start of check
	car.cur_front_car		= false;
	car.cur_rear_car		= false;
	car.left_front_car	= false;
	car.left_rear_car		= false;
	car.right_front_car	= false;
	car.right_rear_car	= false;
	
	double obstacle_d	= 0;
	double obstacle_s	= 0;
	int cur_front_id	= -1;
	// Check for closest surrounding vehicles in all lanes
	for (int i = 0; i< sensor_fusion.size(); i++){
		d = sensor_fusion[i][6];
		// Check for front and rear vehicle in same lane
		if ((obstacle_d > 4*car.lane) && (obstacle_d < 4+4*car.lane)){
			
			obstacle_s = sensor_fusion[i][5];
			// Check if this is closest front car
			if((obstacle_s > car.s) && (obstacle_s < cur_min_front_s)){
				cur_min_front_s = obstacle_s;
				cur_front_id    = i;
				// Set flag for front car
				car.cur_front_car = true;
			}else if(){
				
			}
		}
		// Check for front and rear vehicle in left lane
		else if((obstacle_d > 0) && (obstacle_d < 4*car.lane)){
			
		}
		// Check for front and rear vehicle in right lane
		else if((obstacle_d > 4*car.lane+4) && (obstacle_d < 4*car.lane+8)){
			
		}
	}	
		// Project the obstacle's Frenet position prev_path_size steps into the future
		//obstacle_s += (double)prev_path_size*Ts*obstacle_speed;
		// Check gap between preceding vehicle and ego vehicle
		//(obstacle_s-car.s < cp_inc)
		//double vx = sensor_fusion[i][3];
		//double vy = sensor_fusion[i][4];
		//double obstacle_speed = sqrt(vx*vx + vy*vy);
		/*// FSM for changing lanes
		if(car.lane > 0){
			car.lane -= 1;
		}else if (car.lane < 2){
			car.lane +=1;
		}*/
	
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
    
    ///Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
    // rVehicle is updated if a vehicle is found.
    
    int max_s = -1;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        if (temp_vehicle.lane == this->lane && temp_vehicle.s < this->s && temp_vehicle.s > max_s) {
            max_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}*/

