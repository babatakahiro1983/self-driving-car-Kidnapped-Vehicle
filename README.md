# self-driving-car-Kidnapped-Vehicle


**Initialization**:
- Set a certain number of particles around the GPS measurement. 100 is good enough.
- Initialize all particles to first position (based on estimates of x, y, theta and their uncertainties from GPS)
- Add random Gaussian noise to each particle. We will use following two C++ libraries to accomplish this:
    - [C++ standard library normal distribution](http://en.cppreference.com/w/cpp/numeric/random/normal_distribution) 
    - [C++ standard library random engine](http://www.cplusplus.com/reference/random/default_random_engine/)
- Set each particle weight to 1. 

**Prediction**:
- Predicts the state for the next time step using the Motion model based on Yaw rate and velocity while accounting sensor noise.
- Add measurements to each particle and add random Gaussian noise.
- When adding noise you may find ```std::normal_distribution``` and ```std::default_random_engine``` useful.

**Update Weights**:
- Homogeneous transformation: Transformed Observation (x_map,y_map) = func(x_particle, y_particle, heading_particle, x_obs, y_obs)
- Associate: nearest landmark
- Update weights: multi-varience density function


- **Data Association**: Find the predicted measurement that is closest to each observed measurement and assign the observed measurement to this particular landmark.
- Update the weights of each particle using a mult-variate Gaussian distribution. 
- The observations are given in the VEHICLE'S coordinate system. Your particles are located according to the MAP'S coordinate system. You will need to transform between the two systems. Keep in mind that this transformation requires both rotation AND translation (but no scaling).  
  
**Resample**:
- Resample particles with replacement with probability proportional to their weight.
