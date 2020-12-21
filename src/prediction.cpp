#include "prediction.h"

vector<double> NaiveBayesClassifier(const NaiveBayesParameters_t &param, vector<double> observation)	
{
  /**
   * Once trained, this method is called and expected to return 
   *   a predicted behavior for the given observation.
   * @param observation - a 4 tuple with s, d, s_dot, d_dot.
   *   - Example: [3.5, 0.1, 8.5, -0.2]
   * @output A label representing the best guess of the classifier. Can
   *   be one of "left", "keep" or "right".
   *
   * TODO: Complete this function to return your classifier's prediction
   */
	  
  // Calculate product of conditional probabilities for each label.
  double left_p = 1.0;
  double keep_p = 1.0;
  double right_p = 1.0; 

  // For a feature x and label C with mean mu and standard deviation sigma (computed in training), 
  // the conditional probability can be computed using the formula Naive Bayes formula
  for (int i=0; i<4; ++i) {
    left_p *= (1.0/sqrt(2.0 * M_PI * pow(param.left_stddev[i], 2))) * exp(-0.5*pow(observation[i] - param.left_mean[i], 2)/pow(param.left_stddev[i], 2));
    keep_p *= (1.0/sqrt(2.0 * M_PI * pow(param.keep_stddev[i], 2))) * exp(-0.5*pow(observation[i] - param.keep_mean[i], 2)/pow(param.keep_stddev[i], 2));
    right_p *= (1.0/sqrt(2.0 * M_PI * pow(param.right_stddev[i], 2))) * exp(-0.5*pow(observation[i] - param.right_mean[i], 2)/pow(param.right_stddev[i], 2));
	}

  // Multiply each by the prior
  left_p *= param.left_prior;
  keep_p *= param.keep_prior;
  right_p *= param.right_prior;

  vector<double> probs;
  probs.push_back(left_p);
  probs.push_back(keep_p);
  probs.push_back(right_p);

  return probs;	
}


/* Predict future position of observed cars based on assumption of the car with constant speed */
vector<vector<double>> Predict(vector<vector<double>> observations, int prev_size) {
  vector<vector<double>> predictions;
  
  for(int i=0; i<observations.size(); i++)	{
    vector<double> projected_observation;
    float d = observations[i][6];
    double car_lane = 2;

    if (d<=4) {
      car_lane = 0;
    } else if (d>4 && d<=8) {
      car_lane = 1;
    }
    
    double car_id = observations[i][0];
    double vx = observations[i][3];
    double vy = observations[i][4];
    double velocity = sqrt(vx*vx + vy*vy);	// magnitude of speed
    double s_at_t0 = observations[i][5];
    // Using size of previous points, project s values at next 3 time steps.
    // basically, we looking at car in future  
    double s_at_t1 = s_at_t0 + ((double)prev_size*0.02*velocity);
    double s_at_t2 = s_at_t0 + 2*((double)prev_size*0.02*velocity);
    double s_at_t3 = s_at_t0 + 3*((double)prev_size*0.02*velocity);
    
    projected_observation.push_back(car_id);
    projected_observation.push_back(car_lane);
    projected_observation.push_back(velocity);
    projected_observation.push_back(s_at_t1);
    projected_observation.push_back(s_at_t2);
    projected_observation.push_back(s_at_t3);

    predictions.push_back(projected_observation);
  }
  
  return predictions;
}
