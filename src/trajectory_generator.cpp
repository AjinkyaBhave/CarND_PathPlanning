#include "trajectory_generator.h"

Trajectory_Generator::Trajectory_Generator(){
	mph_to_mps = 0.447;
	max_s = 6945.554;
	
	// Waypoint map to read from
	string map_file_ = "../data/highway_map.csv";
	ifstream in_map_(map_file_.c_str(), ifstream::in);
	string line;
	// Read map file into waypoints vectors
	while (getline(in_map_, line)) {
		istringstream iss(line);
		double x;
		double y;
		float s;
		float d_x;
		float d_y;
		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;
		map_waypoints_x.push_back(x);
		map_waypoints_y.push_back(y);
		map_waypoints_s.push_back(s);
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
	}
}

Trajectory_Generator::~Trajectory_Generator(){}

double Trajectory_Generator::distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int Trajectory_Generator::ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int Trajectory_Generator::NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
	angle = min(2*pi() - angle, angle);

	if(angle > pi()/4)
  {
		closestWaypoint++;
		if (closestWaypoint == maps_x.size()){
			closestWaypoint = 0;
		}
	}
	return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> Trajectory_Generator::getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Trajectory_Generator::getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};
}

void Trajectory_Generator::generate_trajectory (Vehicle car, vector<double>& next_x_vals, vector<double>& next_y_vals, vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s){
	// List of control points that are spaced car.cp_inc apart. 
	// These will be used for spline fitting later
	vector<double> control_points_x;
	vector<double> control_points_y;
	
	// Length of previous path in number of points
	int prev_path_size = previous_path_x.size();
	// Reference starting point for interpolation. 
	// Could be either ego vehicle state or end point of previous path.
	double ref_x; 
	double ref_y; 
	double ref_yaw; 
	double prev_ref_x;
	double prev_ref_y;
	double ref_s;
	
	// If previous path is too small
	if(prev_path_size < 2){
		// Make path locally tangent to car heading
		ref_x = car.x;
		ref_y = car.y;
		ref_yaw = deg2rad(car.yaw);
		prev_ref_x = car.x - cos(car.yaw);
		prev_ref_y = car.y - sin(car.yaw);
		ref_s = car.s;
		//printf("\n using car ref\n");
	}
	// Use previous path's endpoint as reference
	else{
		ref_x = previous_path_x[prev_path_size-1];
		ref_y = previous_path_y[prev_path_size-1];
		prev_ref_x = previous_path_x[prev_path_size-2];
		prev_ref_y = previous_path_y[prev_path_size-2];
		ref_yaw = atan2(ref_y-prev_ref_y, ref_x-prev_ref_x);
		ref_s = end_path_s;
		//printf("\n using prev_path\n");
	}
	control_points_x.push_back(prev_ref_x);
	control_points_x.push_back(ref_x);
	control_points_y.push_back(prev_ref_y);
	control_points_y.push_back(ref_y);
	
	// Add evenly spaced points car.cp_inc apart in Frenet coordinates ahead of starting reference 
	vector<double> next_cp0 = getXY(ref_s+car.cp_inc, 2+4*car.lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_cp1 = getXY(ref_s+2*car.cp_inc, 2+4*car.lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_cp2 = getXY(ref_s+3*car.cp_inc, 2+4*car.lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	
	control_points_x.push_back(next_cp0[0]);
	control_points_x.push_back(next_cp1[0]);
	control_points_x.push_back(next_cp2[0]);
	
	control_points_y.push_back(next_cp0[1]);
	control_points_y.push_back(next_cp1[1]);
	control_points_y.push_back(next_cp2[1]);
	
	// Make vehicle frame as local reference frame for control points
	for (int i = 0; i < control_points_x.size(); i++){
		double shift_x = control_points_x[i] - ref_x;
		double shift_y = control_points_y[i] - ref_y;
		control_points_x[i] = shift_x*cos(-ref_yaw) - shift_y*sin(-ref_yaw);
		control_points_y[i] = shift_x*sin(-ref_yaw) + shift_y*cos(-ref_yaw);
		//printf("%f, ", control_points_x[i]);
	}
	
	// Create a spline from the control points defined
	tk::spline control_spline;
	control_spline.set_points(control_points_x, control_points_y);
	
	// Define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
	// First add unprocessed previous path points from last time step
	for(int i = 0; i < prev_path_size; i++)
	{
		next_x_vals.push_back(previous_path_x[i]);
		next_y_vals.push_back(previous_path_y[i]);
	}
	
	double target_x = car.cp_inc;
	double target_y = control_spline(target_x);
	double target_dist = sqrt(target_x*target_x + target_y*target_y);
	double N = target_dist/(car.Ts*car.ref_vel*MPH_TO_MPS);
	double x_inc = target_x/N;
	double x_offset = 0;
	
	// Complete trajectory generation with spline points
	for(int i=0; i < car.path_size - prev_path_size; i++ ){
		double x_spline = x_offset + x_inc;
		double y_spline = control_spline(x_spline);
		x_offset = x_spline;
		
		// Transform back to global frame from vehicle frame
		double x_global = x_spline*cos(ref_yaw) - y_spline*sin(ref_yaw);
		double y_global = x_spline*sin(ref_yaw) + y_spline*cos(ref_yaw);
		x_global += ref_x;
		y_global += ref_y;
		
		// Add waypoint to path lists
		next_x_vals.push_back(x_global);
		next_y_vals.push_back(y_global);
	}	
}