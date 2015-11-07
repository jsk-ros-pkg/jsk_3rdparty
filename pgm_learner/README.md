pgm_learner
- - -

# What is this

This package provides parameter/structure estimation of bayesian belief network.

**NOTE** Currenlty only parameter estimation is available.

# How to use

```bash
# launch server
roslaunch pgm_learner parameter_estimation.launch
```

```bash
# execute sample client
rosrun pgm_learner discrete_bayesian_parameter_estimation_sample.py
rosrun pgm_learner linear_gaussian_bayesian_parameter_estimation_sample.py
```
