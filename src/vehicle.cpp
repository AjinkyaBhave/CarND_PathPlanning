#include "vehicle.h"

void Vehicle::Vehicle(){
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

void Vehicle::Vehicle(int lane, double max_ref_vel){
	// Start vehicle in KL state in the middle lane stopped
	Vehicle();
	this->lane  			= lane;
	this->max_ref_vel 	= max_ref_vel;
}

void Vehicle::~Vehicle(){}

/*bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
    
    // Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
    // rVehicle is updated if a vehicle is found.
    
    int min_s = this->goal_s;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        if (temp_vehicle.lane == this->lane && temp_vehicle.s > this->s && temp_vehicle.s < min_s) {
            min_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
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

