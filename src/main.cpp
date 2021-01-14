#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "spline.h"
#include "json.hpp"
#include "fsm.h"




const double MAX_VEL = 49.5;
const double MAX_ACC = .224;
const double MAX_DEC = .224;
const int LEFT_LANE = 0;
const int MIDDLE_LANE = 1;
const int RIGHT_LANE = 2;
const int INVALID_LANE = -1;

const int LEFT_LANE_MAX = 4;
const int MIDDLE_LANE_MAX = 8;
const int RIGHT_LANE_MAX = 12;

const int PROJECTION_IN_METERS = 30;

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::cout;

// start in lane 1;
int lane = 1;

// have a reference velocity to target
double ref_vel = 0; //mph
double Des_vel = MAX_VEL; //mph
double Tgt_vel_ego = MAX_VEL;
double Tgt_vel_left = MAX_VEL;
double Tgt_vel_right = MAX_VEL;


bool car_ahead = false;
bool car_left = false;
bool car_left_back = false;
bool car_right = false;
bool car_right_back = false;
bool car_lane_change = false;


enum Long_States { Normal, Follow };
enum Long_Triggers { toFollow, toNormal };
FSM::Fsm<Long_States, Long_States::Normal, Long_Triggers> fsm_long;
const char* StateNames_long[] = { "Normal", "Follow Vehicle In Front" };
void dbg_fsm_long(Long_States from_state, Long_States to_state, Long_Triggers trigger) {
	if (from_state != to_state) {
		std::cout << "Long State Changed To: " << StateNames_long[to_state] << std::endl;
	}
}

enum Lat_States { laneFollow, laneChangeLeft, laneChangeRight };
enum Lat_Triggers { toLeft, toRight, toLaneFollow };
FSM::Fsm<Lat_States, Lat_States::laneFollow, Lat_Triggers> fsm_lat;
const char* StateNames_lat[] = { "Follow lane", "Change to left lane","Change to right lane" };
void dbg_fsm_lat(Lat_States from_state, Lat_States to_state, Lat_Triggers trigger) {
	if (from_state != to_state) {
		std::cout << "Lat State Changed To: " << StateNames_lat[to_state] << std::endl;
	}
}
int main() {
	uWS::Hub h;

	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

	// Waypoint map to read from
	string map_file_ = "../data/highway_map.csv";
	// The max s value before wrapping around the track back to 0
	double max_s = 6945.554;

	std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

	string line;
	while (getline(in_map_, line)) {
		std::istringstream iss(line);
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

	// State Machine Setup
	fsm_long.add_transitions({
		//  from state ,        to state,             triggers,                  guard,                action
		{ Long_States::Normal,  Long_States::Follow,  Long_Triggers::toFollow,   [&] {return true; },  [&] {Des_vel = Tgt_vel_ego; } },
		{ Long_States::Follow,  Long_States::Normal,  Long_Triggers::toNormal,   [&] {return true; },  [&] {Des_vel = MAX_VEL; } },
		{ Long_States::Normal,  Long_States::Normal,  Long_Triggers::toNormal,   [&] {return true; },  [&] {Des_vel = MAX_VEL; } },
		{ Long_States::Follow,  Long_States::Follow,  Long_Triggers::toFollow,   [&] {return true; },  [&] {Des_vel = Tgt_vel_ego; } },
		});
	fsm_long.add_debug_fn(dbg_fsm_long);

	fsm_lat.add_transitions({
		//  from state ,                to state,                     triggers,               guard,                action
		{ Lat_States::laneFollow,       Lat_States::laneChangeLeft,   Lat_Triggers::toLeft,        [&] {return true; },  [&] {lane--; car_lane_change = true; } },
		{ Lat_States::laneFollow,       Lat_States::laneChangeRight,  Lat_Triggers::toRight,       [&] {return true; },  [&] {lane++; car_lane_change = true; } },
		{ Lat_States::laneChangeLeft,   Lat_States::laneFollow,       Lat_Triggers::toLaneFollow,  [&] {return true; },  [&] {car_lane_change = false; } },
		{ Lat_States::laneChangeRight,  Lat_States::laneFollow,       Lat_Triggers::toLaneFollow,  [&] {return true; },  [&] {car_lane_change = false; } },
		});
	fsm_lat.add_debug_fn(dbg_fsm_lat);

	h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
		&map_waypoints_dx, &map_waypoints_dy]
		(uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length,
			uWS::OpCode opCode) {
				// "42" at the start of the message means there's a websocket message event.
				// The 4 signifies a websocket message
				// The 2 signifies a websocket event


				if (length && length > 2 && data[0] == '4' && data[1] == '2') {

  					auto s = hasData(data);



					if (s != "") {
						auto j = json::parse(s);

						string event = j[0].get<string>();



						if (event == "telemetry") {
							// j[1] is the data JSON object

							// Main car's localization Data






							double car_x = j[1]["x"];
							double car_y = j[1]["y"];
							double car_s = j[1]["s"];
							double car_d = j[1]["d"];
							double car_yaw = j[1]["yaw"];
							double car_speed = j[1]["speed"];

							// Previous path data given to the Planner
							auto previous_path_x = j[1]["previous_path_x"];
							auto previous_path_y = j[1]["previous_path_y"];
							// Previous path's end s and d values 
							double end_path_s = j[1]["end_path_s"];
							double end_path_d = j[1]["end_path_d"];

							// Sensor Fusion Data, a list of all other cars on the same side 
							//   of the road.
							auto sensor_fusion = j[1]["sensor_fusion"];

							int Ego_Lane = INVALID_LANE;

							// Determine the lane of the other car
							if (car_d > 0 && car_d < LEFT_LANE_MAX) {
								Ego_Lane = LEFT_LANE;
							}
							else if (car_d > LEFT_LANE_MAX && car_d < MIDDLE_LANE_MAX) {
								Ego_Lane = MIDDLE_LANE;
							}
							else if (car_d > MIDDLE_LANE_MAX && car_d < RIGHT_LANE_MAX) {
								Ego_Lane = RIGHT_LANE;
							}
							

						

							//___________________ Prediction ________________
							car_ahead = false;
							car_left = false;
							car_left_back = false;
							car_right = false;
							car_right_back = false;

							// desired target speed 
							Tgt_vel_ego = MAX_VEL;
							Tgt_vel_left = MAX_VEL;
							Tgt_vel_right = MAX_VEL;

							double Tgt_dist_ego = 100;
							double Tgt_dist_left = 100;
							double Tgt_dist_right = 100;

							// find ref_v to use
							for (int i = 0; i < sensor_fusion.size(); ++i) {

								float target_d = sensor_fusion[i][6];
								int SF_Track_Lane = INVALID_LANE;

								// Determine the lane of the other car
								if (target_d > 0 && target_d < LEFT_LANE_MAX) {
									SF_Track_Lane = LEFT_LANE;
								}
								else if (target_d > LEFT_LANE_MAX && target_d < MIDDLE_LANE_MAX) {
									SF_Track_Lane = MIDDLE_LANE;
								}
								else if (target_d > MIDDLE_LANE_MAX && target_d < RIGHT_LANE_MAX) {
									SF_Track_Lane = RIGHT_LANE;
								}
								if (SF_Track_Lane == INVALID_LANE) {
									continue;
								}

								// Determine the speed of the other car
								double vx = sensor_fusion[i][3];
								double vy = sensor_fusion[i][4];
								double check_speed = sqrt(vx * vx + vy * vy);
								double check_car_s = sensor_fusion[i][5];

								// Estimate the other car's position after executing previous trajectory
								double prev_size = 50;// pred 1 sec
								double car_s_pred = prev_size * 0.02*car_speed/2.24 + car_s;
								double check_car_s_pred = prev_size*0.02 * check_speed + check_car_s;
			
								bool condi1 = (check_car_s > car_s) && (car_s_pred > check_car_s_pred);
								bool condi2 = (check_car_s > car_s) && (car_s_pred > check_car_s);
								bool condi3 = (check_car_s < car_s) && (check_car_s_pred+10 > car_s_pred);
		

								if (SF_Track_Lane == lane) {
									// Other car is in the same lane
									
									if (condi1 || condi2){

										car_ahead |= true;
										if ((check_car_s - car_s) < Tgt_dist_ego) {

											Tgt_vel_ego = check_speed * 2.24;
											Tgt_dist_ego = (check_car_s - car_s);

											/*std::cout << "Tgt_vel_ego : " << Tgt_vel_ego << "\t";
											std::cout << "car_speed : " << car_speed << std::endl;*/

										}
									
									}
								}
								else if (SF_Track_Lane - lane == -1) {
									// Other car is on the left lane
									if (condi1 || condi2) {

										car_left |= true;
										if ((check_car_s - car_s) < Tgt_dist_left) {

											Tgt_vel_left = check_speed * 2.24;
											Tgt_dist_left = (check_car_s - car_s);

										}

									}
									else if (condi3) {

										car_left_back |= true;

									}
								}
								else if (SF_Track_Lane - lane == 1) {
									// Other car is on the right lane
									if (condi1 || condi2) {

										car_right |= true;
										if ((check_car_s - car_s) < Tgt_dist_right) {

											Tgt_vel_right = check_speed * 2.24;
											Tgt_dist_right = (check_car_s - car_s);

										}

									}
									else if (condi3) {

										car_right_back |= true;

									}
								}

								/*std::cout << "Trk_data : " << sensor_fusion[i] << "\t";
								std::cout << "Trk_lane : " << SF_Track_Lane << std::endl;*/
								

							}

							/*std::cout << "car_left : " << car_left << "\t";
							std::cout << "car_ahead : " << car_ahead << "\t";
							std::cout << "car_right : " << car_right << std::endl;*/

							/*std::cout << "car_ahead : " << car_ahead << "\t";
							std::cout << "car_left : " << car_left << "\t";
							std::cout << "car_left_back : " << car_left_back << "\t";
							std::cout << "car_lane_change : " << car_lane_change << std::endl;

							std::cout << "car_ahead : " << car_ahead << "\t";
							std::cout << "car_right : " << car_right << "\t";
							std::cout << "car_right_back : " << car_right_back << "\t";
							std::cout << "car_lane_change : " << car_lane_change << std::endl;

							std::cout << "car_lane_change : " << car_lane_change << "\t";
							std::cout << "lane : " << lane << "\t";
							std::cout << "Ego_Lane : " << Ego_Lane << std::endl;*/

							
							//___________________ Mode Manager (Finite state machine) ________________
							// Lat mode control
							if (car_ahead && !car_left && !car_left_back && !car_lane_change && Ego_Lane!= LEFT_LANE) {

								fsm_lat.execute(Lat_Triggers::toLeft);

							}
							else if (car_ahead && !car_right && !car_right_back && !car_lane_change && Ego_Lane != RIGHT_LANE) {
							
								fsm_lat.execute(Lat_Triggers::toRight);

							}
							else if (car_lane_change && (lane == Ego_Lane)) {

								fsm_lat.execute(Lat_Triggers::toLaneFollow);

							}
							

							// Long mode control
							if (car_ahead) {
								// Execute 'CarAhead' trigger on state machine
								fsm_long.execute(Long_Triggers::toFollow);
							}
							else {
								// Execute 'Clear' trigger on state machine
								fsm_long.execute(Long_Triggers::toNormal);
							}

							

							/*std::cout << "current lane : " << lane << std::endl;
							std::cout << "current vel : " << ref_vel << std::endl;*/



							//_________________ Controller ______________

							//_________________ longitudinal controller ______________

							if (ref_vel < Des_vel) {
								
								ref_vel += MAX_ACC;

							}
							else if (ref_vel > Des_vel) {
								
								ref_vel -= MAX_DEC;

							}


							//___________________ lateral controller ________________

							vector<double> ptsx;
							vector<double> ptsy;

							double ref_x = car_x;
							double ref_y = car_y;
							double ref_yaw = deg2rad(car_yaw);



							json msgJson;

							vector<double> next_x_vals;
							vector<double> next_y_vals;


							int prev_save_index = 35;
							// ptsx, ptsy : (prev point, cur point, LA1, LA2, LA3) total 5 point
							//              of Global X,Y coordinates
							if (previous_path_x.size() < prev_save_index) {

								double prev_car_x = car_x - cos(car_yaw);
								double prev_car_y = car_y - sin(car_yaw);

								ptsx.push_back(prev_car_x);
								ptsx.push_back(car_x);

								ptsy.push_back(prev_car_y);
								ptsy.push_back(car_y);

							}
							else {

								ref_x = previous_path_x[prev_save_index - 1];
								ref_y = previous_path_y[prev_save_index - 1];

								double ref_x_prev = previous_path_x[prev_save_index - 2];
								double ref_y_prev = previous_path_y[prev_save_index - 2];
								ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

								ptsx.push_back(ref_x_prev);
								ptsx.push_back(ref_x);

								ptsy.push_back(ref_y_prev);
								ptsy.push_back(ref_y);

								car_s = end_path_s;

							}

							/*std::cout << "prev_path_length : " << previous_path_x.size() << std::endl;
							std::cout << "prev_path_x" << "\t" << "prev_path_y" << std::endl;

							for (int i = 0; i < previous_path_x.size(); ++i) {

								std::cout << previous_path_x[i] << "\t" << previous_path_y[i] << std::endl;

							}
							cout << "---------------------------" << std::endl;*/


							


							vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
							vector<double> next_wp1 = getXY(car_s + 50, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
							vector<double> next_wp2 = getXY(car_s + 70, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

							ptsx.push_back(next_wp0[0]);
							ptsx.push_back(next_wp1[0]);
							ptsx.push_back(next_wp2[0]);

							ptsy.push_back(next_wp0[1]);
							ptsy.push_back(next_wp1[1]);
							ptsy.push_back(next_wp2[1]);

							/*std::cout << "ptsx" << "\t" << "ptsy" << std::endl;

							for (int i = 0; i < ptsx.size(); ++i) {

								std::cout << ptsx[i] << "\t" << ptsy[i] << std::endl;

							}
							cout << "---------------------------" << std::endl;*/


							// Transform form Global X, Y coordinate to Vehicle local x,y coordinate
							for (int i = 0; i < ptsx.size(); i++) {

								double shift_x = ptsx[i] - ref_x;
								double shift_y = ptsy[i] - ref_y;

								ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
								ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
							}

							// create a spline
							tk::spline s;
							// set (x,y) points to the spline
							s.set_points(ptsx, ptsy);

							for (int i = 0; i < previous_path_x.size(); i++) {

								if (i < prev_save_index) {

									next_x_vals.push_back(previous_path_x[i]);
									next_y_vals.push_back(previous_path_y[i]);

								}
								else {

									break;

								}

							}

							/*std::cout << "next_path_length : " << next_x_vals.size() << std::endl;
							std::cout << "next_path_x" << "\t" << "next_y_vals" << std::endl;

							for (int i = 0; i < next_x_vals.size(); ++i) {

								std::cout << next_x_vals[i] << "\t" << next_y_vals[i] << std::endl;

							}
							cout << "---------------------------" << std::endl;*/

							// Calculate how to break up spline points so that we travel at our desired reference velocity
							double target_x = 60;
							double target_y = s(target_x);
							double target_dist = sqrt((target_x) * (target_x)+(target_y) * (target_y));

							double x_add_on = 0;

							int next_x_vals_size = next_x_vals.size();


							// Fill up the rest of our path planner filling it with previous points,
							// here we will aways output 50 points
							for (int i = 1; i <= 50 - next_x_vals_size; i++) {
								double N = (target_dist / (0.02 * ref_vel / 2.24));
								double x_point = x_add_on + (target_x) / N;
								double y_point = s(x_point);

								x_add_on = x_point;

								double x_ref = x_point;
								double y_ref = y_point;

								// rotate back to normal after ratating it eariler
								x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
								y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

								x_point += ref_x;
								y_point += ref_y;

								next_x_vals.push_back(x_point);
								next_y_vals.push_back(y_point);
							}


							/*std::cout << "next_path_length : " << next_x_vals.size() << std::endl;
							std::cout << "next_path_x" << "\t" << "next_y_vals" << std::endl;

							for (int i = 0; i < next_x_vals.size(); ++i) {

								std::cout << next_x_vals[i] << "\t" << next_y_vals[i] << std::endl;

							}
							cout << "---------------------------" << std::endl;*/



							/*
							double dist_inc = 0.3;
							std::cout << "next_s" << "\t" << "next_d" << "\t" << "next_x" << "\t" << "next_y" << std::endl;
							for (int i = 0; i < 50; ++i) {

								double next_s = car_s + (i+1) * dist_inc;
								double next_d = 6;
								vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);


								std::cout << next_s << "\t" << next_d << "\t" << xy[0] <<"\t" << xy[1] << std::endl;

								next_x_vals.push_back(xy[0]);
								next_y_vals.push_back(xy[1]);
							}
							cout<<"---------------------------"<< std::endl;

				  */



							msgJson["next_x"] = next_x_vals;
							msgJson["next_y"] = next_y_vals;



							auto msg = "42[\"control\"," + msgJson.dump() + "]";

							ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
						}  // end "telemetry" if
					}
					else {
						// Manual driving
						std::string msg = "42[\"manual\",{}]";
						ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
					}
				}  // end websocket if
		}); // end h.onMessage

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
		});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
		char* message, size_t length) {
			ws.close();
			std::cout << "Disconnected" << std::endl;
		});

	int port = 4567;
	if (h.listen(port)) {
		std::cout << "Listening to port " << port << std::endl;
	}
	else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}

	h.run();
}