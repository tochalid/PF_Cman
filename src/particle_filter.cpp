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
static std::default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// REVIEW: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	//Define number of particles
	num_particles = 100;

	//Set up generator for drawing partical states with normal distributionss
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);
	
	//resize vectors for number of particles
	particles.resize(num_particles);
	weights.resize(num_particles);

	//Set initial positions and weights of the particles
	for(int i = 0; i<num_particles; ++i){
		Particle particle;
		particle.id = i;
		particle.x = dist_x(gen);
		particle.y = dist_y(gen);
		particle.theta = dist_theta(gen);
		particle.weight = 1.;
		//Initialize particles and weights vectors
		particles[i] = particle;
		weights[i] = 1.;
	}
	//Set initialized to true
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// REVIEW: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	//Declare the generator to be used to draw from normal distributions

	//Factors to avoid repeated calculations
	double dv = delta_t * velocity;
	double dyaw = delta_t * yaw_rate;
	double q_vyaw = velocity/yaw_rate;

	//Mean positions according to motion model
	double mean_x, mean_y, mean_theta;

	//Calculate mean (x,y,theta) according to motion model
	for(int i = 0; i<num_particles; ++i){
		//Consider small yaw rates
		if(fabs(yaw_rate)<0.001){
			mean_x = particles[i].x + dv * cos(particles[i].theta);
			mean_y = particles[i].y + dv * sin(particles[i].theta);
			mean_theta = particles[i].theta;
		}
		else{
			mean_x = particles[i].x + q_vyaw * (sin(particles[i].theta + dyaw) - sin(particles[i].theta));
			mean_y = particles[i].y + q_vyaw * (cos(particles[i].theta) - cos(particles[i].theta + dyaw));
			mean_theta = particles[i].theta + dyaw;
		}	

		//Generate normal distributions with std dev of x,y,theta
		normal_distribution<double> norm_x(mean_x, std_pos[0]);
		normal_distribution<double> norm_y(mean_y, std_pos[1]);
		normal_distribution<double> norm_theta(mean_theta, std_pos[2]);

		//add noise
		particles[i].x = norm_x(gen);
		particles[i].y = norm_y(gen);
		particles[i].theta = norm_theta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs>& predicted, 
								std::vector<LandmarkObs>& observations) {
	// REVIEW: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	
	//For each predicted measurement
	for(int p = 0; p<predicted.size(); ++p){
		//Initialize minimum distance required to a large number
		double min_distance = 1000;
		double distance = 0;
		//For each predicted measurements find closest landmark in all observations
		for(int l = 0; l<observations.size(); ++l){
			//Calc euclidean distance between predicted and observed measurement
			distance = dist(predicted[p].x, predicted[p].y, observations[l].x, observations[l].y);
			if(distance < min_distance){
				//Add the landmark id (integer) to the predicted measurements
				predicted[p].id = l;
				min_distance = distance;
			}
		}
	}
}

void ParticleFilter::updateWeights(double sensor_range, 
		double std_landmark[], 
		const std::vector<LandmarkObs> &observations, 
		const Map &map_landmarks) {
	// REVIEW: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	vector<double> sense_x;
	vector<double> sense_y;
	vector<int> associations;

	int num_obs = observations.size(); //number of observations of current step
	sense_x.resize(num_obs);
	sense_y.resize(num_obs);
	associations.resize(num_obs);
	vector<LandmarkObs> space_obs; //vector of observations (map coordinates)
	space_obs.resize(num_obs);

	//Precalc to avoid repeated computing when calculating probability density
	double norm = 2*M_PI*std_landmark[0]*std_landmark[1];
	double sigmaX_term = 2*pow(std_landmark[0], 2);
	double sigmaY_term = 2*pow(std_landmark[1], 2);

	//Loop over all particlesprobability density
	vector<LandmarkObs> space_landmarks_in_range; //vector of landmarks in range (map coordinates)
	double dist_to_landmark;
	int n_marks = map_landmarks.landmark_list.size();
	for(int i = 0; i<num_particles; ++i){
		//Remove landmarks in range of every particle
		space_landmarks_in_range.clear();
		//Precalc to avoid repeated computing
		double cos_theta = cos(particles[i].theta);
		double sin_theta = sin(particles[i].theta);

		/*
		use the particle coordinates and heading to transform the car's frame of reference 
		to the map's frame of reference, associate the observations, 
		then use the associated observations in the map domain to update the particle weight
		*/
		//For each observation convert to real space coordinates using Homogenous Transformation
		//transform observation from car frame to map frame (for this particle)
		for(int z = 0; z<num_obs; ++z){
			LandmarkObs actual_obs;
			actual_obs.id = n_marks; //set correlation id
			actual_obs.x = particles[i].x + cos_theta*observations[z].x - sin_theta*observations[z].y;
			actual_obs.y = particles[i].y + sin_theta*observations[z].x + cos_theta*observations[z].y;

			//Set associations
			sense_x[z] = actual_obs.x;
			sense_y[z] = actual_obs.y;
			//Add transformed observations
			space_obs[z] = actual_obs;
		}

		//Match the predicted measurements to landmarks in sensor range
		for(int l=0; l<n_marks; ++l){
			//Check if landmark is in sensor range
			dist_to_landmark = dist(particles[i].x, particles[i].y, map_landmarks.landmark_list[l].x_f, map_landmarks.landmark_list[l].y_f);
				if(dist_to_landmark < sensor_range){
					// landmark association
					LandmarkObs lmark_in_range;
					lmark_in_range.id = map_landmarks.landmark_list[l].id_i;
					lmark_in_range.x = map_landmarks.landmark_list[l].x_f;
					lmark_in_range.y = map_landmarks.landmark_list[l].y_f;
					space_landmarks_in_range.push_back(lmark_in_range);
				} 
		}
		//Call data association using correlation of ids: space_obs to space_landmarks_in_range
		dataAssociation(space_obs, space_landmarks_in_range);

		//Calculate the Multivariate-Gaussian Probability
		double new_weight = 1.;
		double exponent;
		//Calculate new weights
		for(int z = 0; z<num_obs; ++z){
			int lmark_id = space_obs[z].id;
			//Set associations
			associations[z] = space_landmarks_in_range[lmark_id].id;
			//calculate exponent			
			exponent = -(pow((space_obs[z].x - space_landmarks_in_range[lmark_id].x),2)/sigmaX_term 
						 + pow((space_obs[z].y - space_landmarks_in_range[lmark_id].y) ,2)/sigmaY_term);
			//calculate weight using normalization terms and exponent
			new_weight *= (1./norm)*exp(exponent);
		}
		//Set the new weights
		particles[i].weight = new_weight;
		weights[i] = new_weight;
		//Set the associations
		particles[i] = SetAssociations(particles[i], associations, sense_x, sense_y);
	}
}

void ParticleFilter::resample() {
	// REVIEW: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	//random distribution using the particle weights
	discrete_distribution<int> resample_dist(weights.begin(), weights.end());
	std::default_random_engine gen;

	//create a new vector with particles resampled by weight
	vector<Particle> resamples;
	resamples.resize(num_particles);
	for(int i = 0; i<num_particles; ++i){
		resamples[i] = particles[resample_dist(gen)];
	}
	//Set resampled particles to current particles
	particles = resamples;
}

Particle ParticleFilter::SetAssociations(Particle& particle, 
		std::vector<int>& associations, 
        std::vector<double>& sense_x, 
		std::vector<double>& sense_y)
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

	// return associated particle
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
