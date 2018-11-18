#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "vehicle.h"
#include "trajectory_generator.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != string::npos) {
	 return "";
	} else if (b1 != string::npos && b2 != string::npos) {
	 return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

int main() {
	uWS::Hub h;
	// Ego vehicle object
	Vehicle car = Vehicle();
	Trajectory_Generator tg = Trajectory_Generator();
	
	h.onMessage([&car, &tg](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
	 // "42" at the start of the message means there's a websocket message event.
	 // The 4 signifies a websocket message
	 // The 2 signifies a websocket event
	 //auto sdata = string(data).substr(0, length);
	 //cout << sdata << endl;
	 if (length && length > 2 && data[0] == '4' && data[1] == '2') {

		auto s = hasData(data);

		if (s != "") {
		  auto j = json::parse(s);
		  
		  string event = j[0].get<string>();
		  
		  if (event == "telemetry") {
			// j[1] is the data JSON object
			 
			// Main car's localization Data
			car.x = j[1]["x"];
			car.y = j[1]["y"];
			car.s = j[1]["s"];
			car.d = j[1]["d"];
			car.yaw = j[1]["yaw"];
			car.speed = j[1]["speed"];

			// Previous path data given to the Planner
			auto previous_path_x = j[1]["previous_path_x"];
			auto previous_path_y = j[1]["previous_path_y"];
			// Previous path's end s and d values 
			double end_path_s = j[1]["end_path_s"];
			double end_path_d = j[1]["end_path_d"];

			// Sensor Fusion Data, a list of all other cars on the same side of the road.
			auto sensor_fusion = j[1]["sensor_fusion"];

			// List of actual (x,y) waypoints used for trajectory generation
			vector<double> next_x_vals;
			vector<double> next_y_vals;
			
			// Get environment around ego vehicle
			car.get_surrounding_vehicles(sensor_fusion, previous_path_x.size(), end_path_s);
			// Decide next state, new lane, and new ref_vel for ego vehicle
			car.choose_next_state();
			// Generate reference trajectory for ego vehicle to follow
			tg.generate_trajectory(car, next_x_vals, next_y_vals, previous_path_x, previous_path_y, end_path_s);
			
			// Send reference trajectory to simulator to execute
			json msgJson;
			msgJson["next_x"] = next_x_vals;
			msgJson["next_y"] = next_y_vals;

			auto msg = "42[\"control\","+ msgJson.dump()+"]";

			//this_thread::sleep_for(chrono::milliseconds(1000));
			ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			 
		  }
		} else {
		  // Manual driving
		  std::string msg = "42[\"manual\",{}]";
		  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
		}
	 }
	});

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
