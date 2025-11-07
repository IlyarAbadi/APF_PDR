#  Map-aided Adaptive Particle Filter with a Reduced State Space

This is a MATLAB-based map-based mixed filtering pedestrian dead reckoning (PDR) using IMU sensors.

# How to run the code

Currently all the required functions and the main code are all included in one MATLAB file.The main MATLAB function includes helper inner functions to aid with a modular design but also reduce the number of additional files. We have includes all the required MATLAB function inside one larger function for the ease of use. 

In order to run the code the user should modify the paths and tuning parameters in `PDR_PF_2D_param_initialization()`.
The main path to outmost data folder is identified as.
`PDR_PF_2D_param_initialization()` includes other hyperparameters to be tuned. These are included in the following sections. 

# Folder Structure
 The user is accepeted to have the following data folder structure. 

- /outmost_folder
  - /calibration/
    - gyros_calib.mat
    - accel_calib.mat
- /navigation/

Calibration for the accelerometer and gyroscope are NOT REQUIRED. However we recommend performing a calibration for acceleroemeter data. The calibration files should be saved in native MATLAB format (`.mat`) for each triaxial gyroscope and accelerometer (the names of these files are expected to be `accel_calib.mat` and `gyros_calib.mat`). See calibration section for more information below.

The user should also provide IMU filenames as `init_02_IMU_filename`. The first argument is a string that helps that can help the algorithm to parse the IMU data fro a specific input data. Here the user can define their own device (granted that a custom reader is coded inside `PDR_PF_2D_load_IMU` function). 

We have tested our data using Sensor Logger App which is available on Android devices. The Sensor Logger produces accelerometer and gyroscope files in separate files CSV files. When reading the files ensure the data is converted to a matrix of the following format.

`timestamp, accelerometer x (m/s^2), accelerometer y (m/s^2), accelerometer z (m/s^2), gyroscope x (deg/second), gyroscope y (deg/second), gyroscope z (deg/second)`.

The user can write their own file to convert the inputs to the format mentioned above for compatbility with the rest of the algorithm.

# Hyperparameters
Hyperparameters for the code all included in the `PDR_PF_2D_param_initialization()` inner function. User can set the initial particle filter parameters and running particle filter parameters. In particular two variables `init_08_sampling_size` and `init_09_cross_entropy` are important. In the example below initial particle number is set to 2000, initial particle heading is set to a range from -91 to 89. Gyro bias noise standard deviation is set to 0.03, and step length noise standard deviation is set to 0.4. Particle filter about reduction is set to 1000.

```ruby
   init_08_sampling_size   = {2000,{-91:1:-89},{0.03,0.4},1000}; 
```

In the following example, the heading threshold for cross entropy is set to 1.8 degrees and the position threshold hold is set to 3.0. Larger thresholds will cause the algorithm to trigger the reduction earlier, while smaller thresholds will cause the algorithm to trigger reduction later.

```ruby
  init_09_cross_entropy = {1.8,3.0}; 
```

# Calibration (optional)
It is important to note that inclusion of a calibration is not required. A calibration can be removed by setting NO_CALIBRATION in the very begnning of the file. We highly recommend to use the calibration procedure an the code using the method in [https://github.com/IlyarAbadi/IMU_Calibration_WO_Turntable]. The output of the calibration shoud be a 4 by 4 matrix for each sensor, as shown below :
$$
\mathbf{K}_{\text{gyro}} =
\begin{bmatrix}
s_{g_x} & m_{g_{xy}} & m_{g_{xz}} & b_{g_x} \\
m_{g_{yx}} & s_{g_y} & m_{g_{yz}} & b_{g_y} \\
m_{g_{zx}} & m_{g_{zy}} & s_{g_z} & b_{g_z} \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

$$
\mathbf{K}_{\text{acc}} =
\begin{bmatrix}
s_{a_x} & m_{a_{xy}} & m_{a_{xz}} & b_{a_x} \\
m_{a_{yx}} & s_{a_y} & m_{a_{yz}} & b_{a_y} \\
m_{a_{zx}} & m_{a_{zy}} & s_{a_z} & b_{a_z} \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

Although particle filter is a robust estimator, it will suffer from uncalibrated gyroscopes to some extent (especially with a significant gyroscope bias). The algorithm DOES NOT remove the initial bias values from a gyroscope and particle filter can only accomodate up to 3 degrees bias in the tested scenario. 

# Inclusion of static phases 
It is recommend to have 1 second pause in the beginning (the pauses while the user remains stationary). The one second pause is utilized to approximate the axis that corresponds to the vertical axes. The user needs to mention the last epoch (before pause). EXACT starting and the end time of the static pauses are NOT REQUIRED. The user can enter only aproximate values. If user provided 1 second pause and the data rate is 100, then the first number in `init_02_IMU_filename` should be set to 100 (the second number is the index of the final epoch).

# Step length estimation
Step length estimation is important in any PDR. In this code, step length estimation is based on detection of the peaks and the vallies in the vertical direction (parallel to floor) of the accelerometer. In the `PDR_PF_2D_step_detect` function the accelerometer readings are projected onto the vertical direction. We assume that floor is flat and if the smartphone is placed on the floor, there will be an alignment of the floor plane and one of the axes of the acceleroemter (EXACT ALIGNMENT is not required, as the estimated length of the step will be corrected through particle filter). With these assumptions accelerometer measurement are projected onto the vertical direction. and the minimum and the maximum values in this direction are detected. Detecting maximum and minimum require identifying a window `window_size`. This variable is local to `PDR_PF_2D_step_detect` and is set AUTOMATICALLY. The value of this window size can be changed if you receive a [WARNING] that "Number of peaks and vallies are different" in MATLAB promopt. Typically, if the peaks and vallies are different up to 10-14 steps, the algorithm runs without issues, but if this difference increases, it can cause failures. 

The detected epochs of the maximum and the minimum accelerometer values and accelerometer values themselves are passed to `PDR_PF_2D_estimate_length`, where we utilized Weinberg's step length estimation formula. This formula requires setting a constant value. This value is set to k = 0.4.This value is only approximate and user depedent. EXACT values for this parameter is NOT REQUIRED as particle filter through map-updates can handle errors.

Butterworh frequency filtering is utilized to remove high frequency error in the data. Currently a cut-off frequency(`cutoff`) of 8 Hz is utilized. The cut-off frequency can be decreased or increased based on the type of IMU used. 

# Generating initial particles
Generating initial particle happens in few steps. First the algorithm reads CSV file containing the map `PDR_PF_2D_read_CAD`. This is a vectorized data, where each line corresponds to x1,y1,x2,y2 referring to the first and last points on a line-segment. If the CAD model is in the correct WORLD (METRIC) space, then `init_05_map_information` scale should be set to 1. In our case, a scale factor of `4.3197` was used to transfer the CAD scale to correct metric scale. 

Following this step, `PDR_PF_2D_gen_occupy` uses imported map-lines and places them in discretized matrix. The resolution of this map is defined based on `resolution` should be fine (small enough) to perserve all the structures in the map. `grid_D = imdilate(grid,ones(4,4))` line is used to connect the nearby points in the discretized points and avoid empty gaps in the map lines.

The discrete map is utilized to spawn possible set of user position in `PDR_PF_2D_init_occupy`. If, user want to select particles in a specific region of the map, they can do so by setting `lim_x_lower`, `lim_x_upper`, `lim_y_lower`, and `lim_y_upper` values. Otherwise, these values can be set to `inf` and `-inf`. Based on the desired number of particles `par_num_pos`, the function will select particles at regular spaces matching closest to this number. FINALLY, the grid coordinates are transfered to original METRIC values matching the real-dimensions of the CAD model in this function (thus `m_par` is the world scale).

It is important to note that that sampling the user's heading will multiply `par_num_pos`. So the heading sampling step set in  `init_08_sampling_size` (second cell,first parameter) will be multiplied to `par_num_pos`. If user requires the same number of particles as `init_08_sampling_size`, then the user should provide correct heading (second cell, second parameter) and set heading multiplier to one. EXACT knowledge of heading is NOT REQUIRED and approximate values up to 5 degrees should work. If user's is not sure about the heading in this range, then heading multiplier should utilized and heading range should be expanded.

# Adaptive Particle filtering

Particle filtering follows the method mentioned in [REF] which this work is based on. The proposed particle filtering is map-aided, where particles are propogated forward with the help of IMU measurement.

While accelerometer measurements are used to estimate the step-length, gyroscope measurements are uitlized to estimate the yaw angle. The following are the distinction between the proposed particle filter and other particle filters:
  1. The proposed method relies on leveling gyroscope measurements using gravity vector from accelerometer. Since each accelerometer reading can estimate the gravity vector independently, such leveling can be achieved independent of the particle filtering process and thus no additional requirements for state augmentation is required. Keeping the number of the state vector low, will reduce the computational cost of the proposed particle filter. 
  2. The proposed particle filter takes into account the bias in the gyroscope as part of the state variable. Gyroscope bias is a time-varying state which cannot be calibrated prior to the PDR mission for most MEMS based IMUs. 
  3. The proposed particle filter does not sample x and y directions directly and instead samples the step length. Due to the special assumptions about locomotion of a human, the x and y are related to each other through step-length and step direction. Thus, the parameters are not sampled indepdently.

Particle prediction is achieved in `particle_filter_predict`.

The developed algorithm utilizes the map top update the particles. Two types of map updates are included. The main and important map-update checks to see if a particle moves throughout walls defined the 2D plan of the building. Particle updates can be found in `particle_filter_update1` and `particle_filter_update1`. It is important to note that such particle update will only occur if the user takes a step.

After every particle filter update, the developed method utilizes cross entropy distance of particle filter spread and a Gaussian distribution. If this distance is smaller than a certain value, the it means particles are convering and particle numbers can be reduced. Cross entropy test is performed in `PDR_PF_2D_CE_test`. The parameter to tune in this function includes mainly the covariance of the target distribution which is defined with local (to function) parameter `gaussian_cov`. The minimum distance acceptable from this distribution is set using `CE_posisti_threshold` and `CE_heading_threshold`. 

TODO: In current implementation, particle reduction is triggered once. More robust methods can increase the number of the particles if cross entropy distance falls outside of a certain range. 


# Example

![Alt text](/Matlab/RPF_PDR/20251105_213340_Mode_not%20reduced_ParNumInit6000_GyroBias0.03_StepLengthNoise0.4ParReduced1000.gif)


# License
APF_PDR is released under BSD-3 License.
