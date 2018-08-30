// Class to store ego vehicle-related state and parameters

Class Vehicle{
	
	public:
		// Members
		
		// Start state of ego vehicle. KL=0, PLCR=1, PLCL=2, LCR=3, LCL=4
		int state;
		// Starting lane of ego vehicle. Lane closest to the centre line is 0. Middle lane is 1. Right lane is 2.
		int lane;
		// Width of each lane in [m]
		int lane_width;
		// Global coordinates in [m]
		double x;
		double y
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
		// Change in ref_vel in mph to achieve 5 m/s^2 average acceleration
		double ref_vel_dec;
		// Sample time of simulator in seconds
		double Ts;
		// Length of current path in number of points 
		int path_size;
		// Flag to initiate reducing current speed
		bool obstacle_close; 
		
		// Methods
		// Constructors
		Vehicle();
		Vehicle(int lane, double max_ref_vel);
		//Destructor
		virtual ~Vehicle();
		bool get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);
		bool get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);
	
};

