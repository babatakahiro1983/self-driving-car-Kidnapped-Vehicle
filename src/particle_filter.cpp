/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

bool pred_debug_flag = true;
bool update_debug_flag = false;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	// Set the number of particles
	num_particles = 10;
	
	// "gen" is the random engine initialized earlier.
	default_random_engine gen;

	// Normal (Gaussian) distribution for x
	normal_distribution<double> dist_x(x, std[0]);  // std[0]: standard deviation of x [m]	
	// Normal distributions for y
	normal_distribution<double> dist_y(y, std[1]);  // std[1]: standard deviation of y [m]
	// Normal distributions for theta
	normal_distribution<double> dist_theta(theta, std[2]); // std[2]: standard deviation of yaw [rad]

	//intialising weights and particles
	weights = vector<double>(num_particles);
	particles = vector<Particle>(num_particles);

	for (int i = 0; i < num_particles; i++) {
		/*Particle particle;
		particle.id = i;
		particle.x = dist_x(gen);
		particle.y = dist_y(gen);
		particle.theta = dist_theta(gen);
		particle.weight = 1.0;   // all weights to 1.
		particles.push_back(particle);
		weights.push_back(1);

		// Print your samples to the terminal.
		//cout << "Sample " << i + 1 << " " << particle.id << " " << particle.x << " " << particle.y << " " << particle.theta <<" " << particle.weight << endl;
		*/
		particles[i].id = i;
		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);
		particles[i].weight = 1.0;
		weights[i] = 1.0;
		cout << particles[i].id << ", " << particles[i].x << ", " << particles[i].y << ", " << particles[i].theta << endl;
	}
	//cout << "----------" << particles.size() << endl;
	
	// set the initialization control to true
	is_initialized  = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/	
	
	// "gen" is the random engine initialized earlier.
	default_random_engine gen;

	if (pred_debug_flag) {
		cout << "predicted    " << particles[0].x << "  " << particles[0].y << endl;
	}

	for(int i=0; i < num_particles; i++){
		double new_x, new_y, new_theta;
        if (fabs(yaw_rate) <= 0.0001){
			new_x = particles[i].x + velocity*delta_t*cos(particles[i].theta);
			new_y = particles[i].y + velocity*delta_t*sin(particles[i].theta);
			new_theta = particles[i].theta;
		} else {
			new_x = particles[i].x + velocity/yaw_rate * (sin(particles[i].theta+yaw_rate*delta_t) - sin(particles[i].theta));
			new_y = particles[i].y + velocity/yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta+yaw_rate*delta_t));
			new_theta = particles[i].theta + yaw_rate*delta_t;
		}

		// Normal (Gaussian) distribution for x, y, theta
		normal_distribution<double> dist_x(new_x, std_pos[0]);  // std_pos[0]: standard deviation of x [m]	
		normal_distribution<double> dist_y(new_y, std_pos[1]);  // std_pos[1]: standard deviation of y [m]
		normal_distribution<double> dist_theta(new_theta, std_pos[2]); // std_pos[2]: standard deviation of yaw [rad]

		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);
		
	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs> &observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	for(int i = 0; i < observations.size(); i++) {
		double xObs = observations[i].x;
		double yObs = observations[i].y;

		double min_distance = std::numeric_limits<double>::infinity();

		int indexOfPredictedObservationWithMinDistance = 0;
		for(int j = 0; j < predicted.size(); j++){
			double xPredicted = predicted[j].x;
			double yPredicted = predicted[j].y;
			double distance = dist(xObs, yObs, xPredicted, yPredicted);

			if (distance < min_distance)
			{
				min_distance = distance;
				indexOfPredictedObservationWithMinDistance = j;

			}
		}

		//cout <<"Observation: " << i <<" Index: " << indexOfPredictedObservationWithMinDistance << " Distance: " << min_distance << endl;
		observations[i].id = indexOfPredictedObservationWithMinDistance;
  }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	for (int i=0; i< num_particles; i++ ){
		double x_particle= particles[i].x;
		double y_particle= particles[i].y;
		double theta_particle = particles[i].theta;

		/**********************************************************
		 *        coordinates transformations                     *
		 **********************************************************/
		std::vector<LandmarkObs> transformed_observations;
		for (auto obs_lm : observations) {
			LandmarkObs t_obs;
			// transform to map x coordinate
			t_obs.x = x_particle + obs_lm.x * cos(theta_particle) - obs_lm.y * sin(theta_particle);
			// transform to map y coordinate
			t_obs.y = y_particle + obs_lm.x * sin(theta_particle) + obs_lm.y * cos(theta_particle);
			// same id
			t_obs.id = obs_lm.id;

			//transformed_observations.push_back(std::move(t_obs));
			transformed_observations.push_back(t_obs);

			cout << "Observation - " << obs_lm.id << ":" << obs_lm.x << ", " << obs_lm.y << endl;
        }


		/**********************************************************
		 *         Landmark predictions ----                      *
		 **********************************************************/
		std::vector<LandmarkObs> predicted_landmarks;
		for (auto lm : map_landmarks.landmark_list) {
			auto dx = lm.x_f - x_particle;
			auto dy = lm.y_f - y_particle;
			auto dist = sqrt(dx*dx + dy*dy);

			// Add only if in range
			if(dist < sensor_range){
				double rotatedx = dx*cos(theta_particle) + dy*sin(theta_particle);
				double rotatedy = -dx*sin(theta_particle) + dy*cos(theta_particle);

				LandmarkObs lm_pred;
				//lm_pred.x = lm.x_f;
				//lm_pred.y = lm.y_f;
				lm_pred.x = rotatedx;
				lm_pred.y = rotatedy;
				lm_pred.id = lm.id_i;

				predicted_landmarks.push_back(lm_pred);
			}
		}


  
		/**********************************************************
		 *         data association - nearest                     *
		 **********************************************************/
		// Stores index of associated landmark in the observation
		dataAssociation(predicted_landmarks, transformed_observations);


		/**********************************************************
		 *    Update weights: multi-varience density function     *
		 **********************************************************/
		double particleWeight = 1.0f;

		for(int k = 0; k < transformed_observations.size(); k++)
		{
			double x = transformed_observations[k].x;
			double y = transformed_observations[k].y;
			int indexInPredictedArray = transformed_observations[k].id;
			double mean_x = predicted_landmarks[indexInPredictedArray].x;
			double mean_y = predicted_landmarks[indexInPredictedArray].y;

			double deltax = x - mean_x;
			double deltay = y - mean_y;
			double deltax_squared = deltax*deltax;
			double deltay_squared = deltay*deltay;
			double sigma_x_squared = std_landmark[0]*std_landmark[0]; 
			double sigma_y_squared = std_landmark[1]*std_landmark[1];

			double numerator = exp(-0.5*((deltax_squared/sigma_x_squared) + (deltay_squared/sigma_y_squared)));
			double denominator = 2.0*M_PI*std_landmark[0]*std_landmark[1];

			//double partialWeight = numerator/denominator;
			// particleWeight *= partialWeight;
			particleWeight *= numerator/denominator;
		}

		
		particles[i].weight = particleWeight;
    	weights[i] = particleWeight;

	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	default_random_engine gen;
	discrete_distribution<int> distribution(weights.begin(), weights.end());
	
	vector<Particle> resample_particles;

	for(int i=0; i < num_particles; i++){
		resample_particles.push_back(particles[distribution(gen)]);
	}

	particles = resample_particles;


}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
