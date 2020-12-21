#ifndef BEHAVIOR_PLANNING_H
#define BEHAVIOR_PLANNING_H

#include <algorithm>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>
#include <cmath>
#include <numeric>
#include <map>

// enable printing state change of car
#define DEBUGGING

using std::vector;
using std::cout;
using std::endl;
using std::string;

typedef enum BehaviorState {
  kReady = 0,
  kKeepLane,
  kChangeLaneLeft,
  kChangeLaneRight,
  kPrepareLaneChangeLeft,
  kPrepareLaneChangeRight
  
} BehaviorState_t;

vector<BehaviorState_t> SuccessorStates(BehaviorState_t current_state, int current_lane);

double CostFunctionDistance(double distance);

double CostFunctionSpeed(double car_velocity, double obs_velocity, double distance);

string StateName(BehaviorState_t current_state);

double CalculateCost(double car_s, double car_speed, BehaviorState_t state, vector<vector<double>> predictions);

BehaviorState_t StateTransition(vector<vector<double>> predictions, BehaviorState_t current_state, double car_s, double car_speed, int current_lane);

int LaneChange(BehaviorState_t current_state, int current_lane, bool& changing_lane) ;

bool TooClose(vector<vector<double>> predictions, double car_s, int current_lane);

double AdjustVelocity(bool is_close, double velocity);



#endif