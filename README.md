# self-driving-car-Kidnapped-Vehicle


**Initialization**:
- Set a certain number of particles around the GPS measurement. 100 is good enough.
- Initialize all particles to first position (based on estimates of x, y, theta and their uncertainties from GPS)
- Add random Gaussian noise to each particle. We will use following two C++ libraries to accomplish this:
    - [C++ standard library normal distribution](http://en.cppreference.com/w/cpp/numeric/random/normal_distribution) 
    - [C++ standard library random engine](http://www.cplusplus.com/reference/random/default_random_engine/)
- Set each particle weight to 1. 
- sample particles data:
```
Sample 1 0 6.29261 2.25289 0.0109327 1
Sample 2 1 6.00315 1.72504 0.0180484 1
Sample 3 2 6.33928 2.18636 0.0318468 1
Sample 4 3 6.17121 2.10781 0.0143172 1
```

**Prediction**:
- Predicts the state for the next time step using the Motion model based on Yaw rate and velocity while accounting sensor noise FOR each particle. _In other words, the 100 particles are moving_.
- Add measurements to each particle and add random Gaussian noise.
- When adding noise you may find ```std::normal_distribution``` and ```std::default_random_engine``` useful.
- sameple prediction data:
```
Sample 1 0 7.03332 2.57475 0.288935 0
Sample 2 0 7.07999 2.50822 0.309849 0
Sample 3 0 7.3762 2.33111 0.281731 0
Sample 4 0 6.78916 2.13764 0.26845 0
```

**Update Weights**:
- Homogeneous transformation: Transformed Observation (x_map,y_map) = func(x_particle, y_particle, heading_particle, x_obs, y_obs)
- Associate: nearest landmark
- Update weights: multi-varience density function


- **Data Association**: Find the predicted measurement that is closest to each observed measurement and assign the observed measurement to this particular landmark.
- Update the weights of each particle using a mult-variate Gaussian distribution. 
- The observations are given in the VEHICLE'S coordinate system. Your particles are located according to the MAP'S coordinate system. You will need to transform between the two systems. Keep in mind that this transformation requires both rotation AND translation (but no scaling).  
  
**Resample**:
- Resample particles with replacement with probability proportional to their weight.
