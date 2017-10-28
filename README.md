# self-driving-car-Kidnapped-Vehicle


**Initialization**:
- Set the number of particles. 100 is good enough.
- Initialize all particles to first position (based on estimates of x, y, theta and their uncertainties from GPS)
- Set all weights to 1. 
- Add random Gaussian noise to each particle. We will use following two C++ libraries to accomplish this:
    - [C++ standard library normal distribution](http://en.cppreference.com/w/cpp/numeric/random/normal_distribution) 
    - [C++ standard library random engine](http://www.cplusplus.com/reference/random/default_random_engine/)

**Prediction**:
- Prediction predicts the state for the next time step using the process model.
- Add measurements to each particle and add random Gaussian noise.
- When adding noise you may find std::normal_distribution and std::default_random_engine useful.

**Update Weights**:
- **Data Association**: Find the predicted measurement that is closest to each observed measurement and assign the observed measurement to this particular landmark.
- Update the weights of each particle using a mult-variate Gaussian distribution. 
- The observations are given in the VEHICLE'S coordinate system. Your particles are located according to the MAP'S coordinate system. You will need to transform between the two systems. Keep in mind that this transformation requires both rotation AND translation (but no scaling).  
  
**Resample**:
- Resample particles with replacement with probability proportional to their weight.
