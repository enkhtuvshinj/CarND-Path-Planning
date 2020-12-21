#include "behavior.h"

vector<BehaviorState_t> SuccessorStates(BehaviorState_t current_state, int current_lane)	{
  vector<BehaviorState_t> states;
  
  switch(current_state) {
    case kReady:
      states.push_back(kReady);
      states.push_back(kKeepLane);
      break;
    case kKeepLane:
      states.push_back(kKeepLane);
      if(current_lane==0) {         // if the most left lane, only change to right
        states.push_back(kPrepareLaneChangeRight);
      } else if(current_lane==2)  { // if the most right lane, only change to left
        states.push_back(kPrepareLaneChangeLeft);
      } else  {
        states.push_back(kPrepareLaneChangeLeft);
        states.push_back(kPrepareLaneChangeRight);
      }
      break;
    case kChangeLaneLeft:
      states.push_back(kChangeLaneLeft);
      states.push_back(kKeepLane);
      break;
    case kChangeLaneRight:
	    states.push_back(kChangeLaneRight);
      states.push_back(kKeepLane);
      break;
    case kPrepareLaneChangeLeft:
      states.push_back(kKeepLane);
      states.push_back(kChangeLaneLeft);
      break;
    case kPrepareLaneChangeRight:
      states.push_back(kKeepLane);
      states.push_back(kChangeLaneRight);
      break;
  }
  
  return states;
}


string StateName(BehaviorState_t current_state)  {
  string name = "None";
    switch(current_state) {
    case kReady:
      name = "kReady";
      break;
    case kKeepLane:
      name = "kKeepLane";
      break;
    case kChangeLaneLeft:
      name = "kChangeLaneLeft";
      break;
    case kChangeLaneRight:
      name = "kChangeLaneRight";
      break;
    case kPrepareLaneChangeLeft:
      name = "kPrepareLaneChangeLeft";    
      break;
    case kPrepareLaneChangeRight:
      name = "kPrepareLaneChangeRight";
      break;
  }
  return name;
}


/* 
  double CostFunctionVelocity(double velocity)	{
	double cost = 1.0;
	double stop_cost = 0.8;
	double buffer_v = 0.5;
	double speed_limit = 50;
	double target_speed = speed_limit - buffer_v;
	
	if(velocity <= target_speed) {
	  cost = stop_cost*(target_speed - velocity)/target_speed;
	} else if (target_speed < velocity && velocity < speed_limit) {
	  cost = (velocity - target_speed)/buffer_v;
	} 
  
  return cost;
}


double CostFunctionDistance(double distance)	{
	double cost = 1.0;
  double m =-0.4987606239116668; 
  double b = 3.0024787521766663;

	if (distance > 4 && distance < 6) {
	  cost = m*distance + b;
	} else if (distance >= 6) {
	  cost = 1/exp(distance);
	}
  
  return cost;
}

double CostFunctionRatio(double car_velocity, double obs_velocity, double distance)	{
  if(car_velocity!=0 && distance!=0) {
    return std::abs(pow(1.001, obs_velocity/car_velocity) - 1.0/distance);
  } else {
    return 5.0;
  }
} 
*/

/* Calculate cost for distance in Frenete coordinate between cars */
double CostFunctionDistance(double distance)	{
	if (distance < -5) {
	  return (1./exp(distance*distance/250.0) + std::log2(std::abs(distance)/3.0)/distance);
	} else {
	  return (1./exp(distance*distance/250.0));
	}
}

/* Calculate cost for speed ratio based on position of car relative to ego car.
 * If car is behind and safe distance is at least 2m away, then compute cost */
double CostFunctionSpeed(double car_velocity, double obs_velocity, double distance)	{
  double ratio = obs_velocity/car_velocity;
  if(car_velocity>0)  {
    if(distance>=0) {
      return std::sin(1 - ratio);       
    } else if(distance<-2) {  // if not too close consider
      return (-std::sin(1 - ratio));
    } else {
      return 1;
    }
  } else {
    return 1;
  }  
}

/* Find maximum cost for the given lane */
double CalculateCost(double car_s, double car_speed, BehaviorState_t state, vector<vector<double>> predictions) {
  vector<double> costs;
  int n = predictions.size();
  
  if(!predictions.empty())  {
    for (int i = 0; i < n; ++i) {
      double cost = 0.0;
      double predicted_s = predictions[i][3];
      double predicted_velocity = predictions[i][2];

      cost += CostFunctionDistance(predicted_s - car_s);
      cost += CostFunctionSpeed(car_speed, predicted_velocity, (predicted_s - car_s));

      costs.push_back(cost);
    }
    
    return *max_element(begin(costs), end(costs));
  }
  return 0.0;
}

/* Choose state with the mimimum cost */
BehaviorState_t StateTransition(vector<vector<double>> predictions, BehaviorState_t current_state, double car_s, double car_speed, int current_lane)	
{
  vector<BehaviorState_t> states = SuccessorStates(current_state, current_lane);
  double cost;
  vector<double> costs;

  for (vector<BehaviorState_t>::iterator it = states.begin(); it != states.end(); ++it) {
    vector<vector<double>> vehicles_in_lane;
    
    if(*it==kChangeLaneRight || *it==kPrepareLaneChangeRight) {
      std::copy_if(predictions.begin(), predictions.end(), std::back_inserter(vehicles_in_lane), [current_lane](const vector<double> c) { return c[1] == (current_lane+1); });
    } else if((*it==kChangeLaneLeft || *it==kPrepareLaneChangeLeft))  {
      std::copy_if(predictions.begin(), predictions.end(), std::back_inserter(vehicles_in_lane), [current_lane](const vector<double> c) { return c[1] == (current_lane-1); });
    } else {
      std::copy_if(predictions.begin(), predictions.end(), std::back_inserter(vehicles_in_lane), [current_lane](const vector<double> c) { return c[1] == current_lane; });
    }
    
    cost = CalculateCost(car_s, car_speed, *it, vehicles_in_lane);

#if defined(DEBUGGING)   
    cout << StateName(*it) << ":" << cost << " ; ";
#endif

    costs.push_back(cost);
  }
  
  vector<double>::iterator best_cost = min_element(begin(costs), end(costs));
  int best_idx = distance(begin(costs), best_cost);

#if defined(DEBUGGING)
  cout << endl << "selected: " << StateName(states[best_idx]) << endl;
#endif

  return states[best_idx];
}

/* Change lane based on state change */
int LaneChange(BehaviorState_t current_state, int current_lane, bool& changing_lane)  
{
  int lane = current_lane;
  if(!changing_lane)  {
    if(current_state == kChangeLaneLeft && current_lane>0)  {
      lane -=1;
      changing_lane=true;
    } else if(current_state == kChangeLaneRight && current_lane<2) {
      lane +=1;
      changing_lane=true;
    }
  }
  
  return lane; 
  
}

/* Check whether any car on same lane is too close */
bool TooClose(vector<vector<double>> predictions, double car_s, int current_lane) {
  vector<vector<double>> in_lane;
  std::copy_if(predictions.begin(), predictions.end(), std::back_inserter(in_lane), [current_lane](const vector<double> c) { return c[1] == current_lane; });
  
  for(int i=0; i<in_lane.size(); i++) {
    if((in_lane[i][3] > car_s) && (in_lane[i][3] - car_s)<30) {
      return true;
    }
  }
  
  return false;
}


/* Adjust speed of ego car if there is too close car */
double AdjustVelocity(bool too_close, double velocity)  {
  double ref_vel = velocity;
  
    if(too_close)  {
      	// if car in front is too close, slowly decrease speed
				ref_vel -= 0.224;	
    } else if (ref_vel < 49.5) {
      	// if no car in front, slowly increase speed
				ref_vel += 0.224;	
    }
  
  return ref_vel;
}

