
# FeatureExtraction_DataTransforms_TransformClassLibrary

<!--
The following template is based on:
Best-README-Template
Search for this, and you will find!
>
<!-- PROJECT LOGO -->
<br />
  <h2 align="center"> FeatureExtraction_DataTransforms_TransformClassLibrary
  </h2>

  <pre align="center">
    <img src=".\Images\Vehicle transform.jpg" alt="main transforms picture" width="577" height="246.67">
  </pre>

  <p align="center">
  This is the main Transform class library that contains transformation operations typically needed for cartesian data processing.
    <br />
  </p>
</p>

***

<!-- TABLE OF CONTENTS -->
<details open="open">
  <summary><h2 style="display: inline-block">Table of Contents</h2></summary>
  <ol>
    <li>
      <a href="#about">About</a>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="structure">Repo Structure</a>
      <ul>
        <li><a href="#directories">Top-Level Directories</li>
        <li><a href="#dependencies">Dependencies</li>
      </ul>
    </li>
    <li><a href="#functions">Functions</li>
        <ul>
          <li><a href="#fcn_transform_predictwheelvelocity">fcn_transform_predictWheelVelocity - Predicts wheel velocity given the vehicle's angular velocity</li>
          <li><a href="#fcn_transform_encodercounts">fcn_transform_encoderCounts - Calculates encoder counts given the wheel's velocity</li>
          <li><a href="#fcn_transform_determinesensortypeorvehicle">fcn_Transform_determineSensorTypeOrVehicle - Used for determining sensor type.</li>
          <li><a href="#fcn_transform_setperturbationtosensorpose">fcn_Transform_setPerturbationToSensorPose - Used to set the perturbations to sensor pose.</li>
          <li><a href="#fcn_transform_determinetransformmatrix">fcn_Transform_determineTransformMatrix - Used to determine the transform matrix.</li>
          <li><a href="#fcn_transform_findvehicleposeinenu">fcn_Transform_findVehiclePoseinENU - Used to find the vehicle pose in ENU coordinates.</li>
          <li><a href="#fcn_transform_sensorcoordtoenu">fcn_Transform_SensorCoordToENU - Transforms sensor readings from sensor coordinates to ENU coordinates</li>
          <li><a href="#fcn_transform_enutosensorcoord">fcn_Transform_ENUToSensorCoord - Transforms the coordinates of a point from ENU coordinates to the sesnor's coordinates </li>
        </ul>
      </ul>
    <li><a href="#usage">Usage</a></li>
     <ul>
     <li><a href="#general-usage">General Usage</li>
     </ul>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>

***

<!-- ABOUT -->
## About

<!--[![Product Name Screen Shot][product-screenshot]](https://example.com)-->

The purpose of this set of functions is to process cartesian data by doing transformations between coordinate systems: ENU coordinates, sensor coordinates, and vehicle-body coordinates, and by calculating wheel velocities and encoder readings. 

<a href="#featureextraction_datatransforms_transformclasslibrary">Back to top</a>

***

<!-- GETTING STARTED -->
## Getting Started

To get a local copy up and running follow these simple steps.

### Installation

1. Make sure to run MATLAB 2020b or higher. Why? The "digitspattern" command used in the DebugTools utilities was released late 2020 and this is used heavily in the Debug routines. If debugging is shut off, then earlier MATLAB versions will likely work, and this has been tested back to 2018 releases.

2. Clone the repo

   ```sh
   git clone https://github.com/ivsg-psu/FeatureExtraction_DataTransforms_TransformClassLibrary
   ```
3. Run the main code in the root of the folder (script_demo_DataTransforms.m). This will download the required utilities for this code, unzip the zip files into a Utilities folder (.\Utilities), and update the MATLAB path to include the Utility locations. This install process will only occur the first time. Note: to force the install to occur again, delete the Utilities directory

4. Confirm it works! Run script_demo_DataTransforms. If the code works, the script should run without errors. This script produces numerous example images such as those in this README file.

<a href="#featureextraction_datatransforms_transformclasslibrary">Back to top</a>

***

<!-- STRUCTURE OF THE REPO -->
### Directories

The following are the top level directories within the repository:
<ul>
 <li>Functions folder: Contains all functions and their test scripts.</li>
 <li>Utilities: Dependencies that are utilized but not implemented in this repository are placed in the Utilities directory. These can be single files but are most often other cloned repositories.</li>
 <li>Images folder: Contains images used in the README file.</li>
</ul>

<a href="#featureextraction_datatransforms_transformclasslibrary">Back to top</a>

***

### Dependencies

* [Errata_Tutorials_DebugTools](https://github.com/ivsg-psu/Errata_Tutorials_DebugTools) - The DebugTools repo is used for the initial automated folder setup, and for input checking and general debugging calls within subfunctions. The repo can be found at: <https://github.com/ivsg-psu/Errata_Tutorials_DebugTools>

<a href="#featureextraction_datatransforms_transformclasslibrary">Back to top</a>

The dependencies are automatically installed by running the root master script (script_demo_DataTransforms.m).

***

<!-- FUNCTION DEFINITIONS -->
## Functions

#### **fcn_transform_predictWheelVelocity**


This function predicts the velocity of the two rear wheels of the vehicle.

**FORMAT:**
```MATLAB
predicted_values = fcn_transform_predictWheelVelocity(pos_rear_left,pos_rear_right,chassis_w,chassis_v)
```

**INPUTS:**

pos_rear_left: [x,y,z] position of rear left tire in meters. Expected value is [0, d, 0], where d is the distance from middle of rear vehicle axle to middle of wheel.

pos_rear_right: [x,y,z] position of rear right tire in meters. Expected value is [0, - d, 0], where d is the distance from middle of rear vehicle axle to the middle of wheel.

chassis_w: angular velocity of vehicle chassis in rad/s; vector; expected input is an array, [w_x, w_y,w_z], where w_x, w_y, and w_z are the components of the chassis' angular velocity.

chassis_v: linear velocity of vehicle in m/s; vector; expected input is an array, [v_x, v_y, v_z], where v_x, v_y, v_z are components of the chassis' linear velocity.
            
**OUTPUTS:**

wheel_v_rear_left: velocity of rear left wheel in m/s; vector, [V_x, V_y, V_z], where V_x, V_y, V_z are the components of the linear velocity of the rear left wheel. 

wheel_v_rear_right: velocity of rear right wheel in m/s; vector, [V_x, V_y, V_z], where V_x, V_y, V_z are the components of the linear velocity of the rear left wheel. 
      
**Dependencies:**

No dependencies 

**Examples:**

See the script: script_test_fcn_transform_predictWheelVelocity
for a full test suite.

<a href="#featureextraction_datatransforms_transformclasslibrary">Back to top</a>

***
#### **fcn_transform_encoderCounts**

This function calculates the number of encoder coutns for each wheel.

**FORMAT:**

```MATLAB
[discrete_encoder_count_rear_left, discrete_encoder_count_rear_right] = fcn_transform_encoderCounts(wheel_velocity_rear_left,wheel_velocity_rear_right,wheel_radius,initial_counts_rear_left,initial_counts_rear_right,delta_time,counts_per_revolution)
```

**INPUTS:** 

wheel_velocity_rear_left = An array of rear left wheel velocities in m/s.
Expected input is [v_1,v_2,v_3,...],
where each velocity corresponds to a
time step.

wheel_velocity_rear_right = An array of rear right wheel velocities in m/s.
Expected input is [v_1,v_2,v_3,...],
where each velocity corresponds to a
time step.

wheel_radius = Radius of wheel in meters.

initial_counts_rear_left = Initial rear left encoder counts before data
collection started. [counts]

initial_counts_rear_right = Initial rear left encoder counts before data
collection started. [counts]

delta_time = Encoder time step in seconds.

counts_per_revolution = Encoder default number of counts per
revolution. [counts/rev.]

**OUTPUTS:**

discrete_encoder_count_rear_left = An array of the calculated
discrete encoder counts for the
rear left wheel encoder. [counts]

discrete_encoder_count_rear_right = An array of the calculated
discrete encoder counts for the
rear right wheel encoder. [counts]
      
**Dependencies:**

No dependencies 

**Examples:**

See the script: script_test_fcn_transform_encoderCounts
for a full test suite.

<a href="#featureextraction_datatransforms_transformclasslibrary">Back to top</a>

***

#### **fcn_Transform_determineSensorTypeOrVehicle**

This function determines the type of sensor or vehicle based on user inputs. The function takes a string as the input and outputs the type of the sensor/vehicle based on the keyword in the input string

**FORMAT:**

```MATLAB
sensor_or_vehicle = fcn_Transform_determineSensorTypeOrVehicle(type_of_sensor_or_vehicle)
```
**INPUTS:**

type_of_sensor_or_vehicle: sensor string or vehicle string

**OUTPUTS:**

sensor: a string listing the data type, one of:
'sicklidarrear', 'velodyne_lidar_rear', 'rightGPS_rear', 'leftGPSrear', 'GPS_hemisphere_sensorplatform_rear', 'vehicle', 'all'.

Note: If the sensor is not recognized, it lists 'other'.

**Dependencies:**

No dependencies 

**Examples:**

See the script: script_test_fcn_Transform_determineSensorTypeOrVehicle
for a full test suite.

<a href="#featureextraction_datatransforms_transformclasslibrary">Back to top</a>

***
#### **fcn_Transform_setPerturbationToSensorPose**

This function takes sensor_or_vehicle string and the perturbation in
sensor pose  as the inputs and outputs a structure of arrays of
perturbation. Each array represents the pertubation of sensor pose for a
different sensor. 

**METHOD:** 

STEP 1: Determine the "sensor" using sensor_or_vehicle string (input)

STEP 2: Add perturbation to the sensor's position and orientation based 
on perturbation_in_sensorPose_relative_to_SensorPlatform

**FORMAT:**

```MATLAB
sensorPose_Perturbation = fcn_Transform_setPerturbationToSensorPose(sensor_or_vehicle,perturbation_in_sensorPose_relative_to_SensorPlatform)
```
**INPUTS:**

sensor_or_vehicle: Perturbation is added to the position and 
orientation of this "sensor_or_vehicle" 


perturbation_sensor: [x_Perturbation,y_Perturbation,z_Perturbation,
roll_Perturbation,pitch_Perturbation,yaw_Perturbation]

Perturbation in the position of the vehicle:

<ul>
x_Perturbation: This is the perturbation of the sensor in the x
direction, in cm, relative to the sensor platform coordinates.

y_Perturbation: This is the perturbation of the sensor in the y
direction, in cm, relative to the sensor platform coordinates.

z_Perturbation: This is the perturbation of the sensor in the z
direction, in cm, relative to the sensor platform coordinates.
</ul>

Perturbation IN THE orientation of the vehicle - Follows ISO 
convention

<ul>
roll_Perturbation: The perturbation of the sensor orientation 
about its x-axis 

pitch_Perturbation: The perturbation of the sensor orientation 
about its y-axis 

yaw_Perturbation: The perturbation of the sensor orientation 
about its z-axis 
</ul>

**OUTPUTS:**

sensorPose_Perturbation: a structure of arrays. Each array
corresponds to the perturbation added to the sensor.

**Dependencies:**

No dependencies 

**Examples:**

See the script: script_demo_Transforms 

<a href="#featureextraction_datatransforms_transformclasslibrary">Back to top</a>

***

#### **fcn_Transform_determineTransformMatrix**

This function takes vehicle parameters, sensor pose parameters,
sensor_or_vehicle string, vehiclePose_ENU perturbation_in_sensorPose_relative_to_SensorPlatform as the inputs and
outputs a transform matrix, which is later used to transform the sensor
coordinates to ENU coordinates, and ENU coordinates to sensor coordinates

**METHOD:** 

Step 1: Assign the parameters to the vehicle and the sensors

Step 2: The vehicle and sensors are created in the shapes of cubes.

Step 3: If there are any perturbations in the position or orientation of
the sensors, they are added to the sensors before moving them to
the correct locations

Step 4: Move the sensors to the corresponding locations by incorporating
the perturbations

Step 5: Set the pose of the vehicle based on the vehiclePose_ENU
(input)

Step 6: Find the transform matrices of the sensors and vehicle based on
sensor_or_vehicle string by multiplying the translation and 
rotation transform matrices of sensor and vehicle

**FORMAT:**

```MATLAB
transform_Matrix = fcn_Transform_determineTransformMatrix(vehicleParameters, sensorPoseParameters, sensor_or_vehicle, vehiclePose_ENU, perturbation_in_sensorPose_relative_to_SensorPlatform, fig_num)
```
**INPUTS:**

vehicleParameters: The structure of all vehicle parameters

This structure includes dimensions of the vehicle and sensors, 
and the offsets of all the points to locate the origin. 


sensorPoseParameters: The structure of all sensor pose parameters

This structure includes the position and orientation of the 
sensors relative to the vehicle and the rear GPS Hemisphere sensor platform


sensor_or_vehicle: transform matrix of this "sensor_or_vehicle" is
generated to transform the "sensor_or_vehicle" 
coordinates to ENU and vice versa      


vehiclePose_ENU: [x,y,z,roll,pitch,yaw]

position of the vehicle:
<ul>
x: translates the vehicle in the x direction relative to
ENU coordinates

y: translates the vehicle in the y direction relative to
ENU coordinates

z: translates the vehicle in the z direction relative to
ENU coordinates
</ul>

orientation of the vehicle - Follows ISO convention

<ul>
roll: rotates the vehicle about its x-axis relative to ENU
coordinates (the vehicle's orientation changes relative to
the Earth's surface)

pitch: rotates the vehicle about its y-axis relative to ENU
coordinates (the vehicle's orientation changes relative to
the Earth's surface)

yaw: rotates the vehicle about its z-axis relative to ENU
coordinates (the vehicle's orientation changes relative to
the Earth's surface)
</ul>

perturbation_sensor: [x_Perturbation,y_Perturbation,z_Perturbation,
roll_Perturbation,pitch_Perturbation,yaw_Perturbation]

Perturbation in the position of the vehicle:

<ul>
x_Perturbation: This is the perturbation of the sensor in the x
direction, in cm, relative to the sensor platform coordinates.

y_Perturbation: This is the perturbation of the sensor in the y
direction, in cm, relative to the sensor platform coordinates.

z_Perturbation: This is the perturbation of the sensor in the z
direction, in cm, relative to the sensor platform coordinates.
</ul>

Perturbation IN THE orientation of the vehicle - Follows ISO 
convention

<ul>
roll_Perturbation: The perturbation of the sensor orientation 
about its x-axis 

pitch_Perturbation: The perturbation of the sensor orientation 
about its y-axis 

yaw_Perturbation: The perturbation of the sensor orientation 
about its z-axis 
</ul>

(OPTIONAL INPUTS)

fig_num: The figure is plotted if fig_num is entered as the input. 

**OUTPUTS:**

transform_Matrix: transform matrix to transform the sensor
coordinates to ENU coordinates and vice versa

**Dependencies:**

No dependencies 

**Examples:**

See the script: script_test_fcn_Transform_determineTransformMatrix.m for a full test suite.

<a href="#featureextraction_datatransforms_transformclasslibrary">Back to top</a>

***

#### **fcn_Transform_findVehiclePoseinENU**

This function takes two GPS Antenna centers, GPSLeft_ENU and 
GPSRight_ENU, in ENU coordinates as a (1 x 3) vector representing 
[x, y, z] in meters, PITCH_vehicle_ENU in degrees and the sensor 
mount's offset relative to the vehicle's origin as

[SensorMount_offset_x_relative_to_VehicleOrigin, 
SensorMount_offset_y_relative_to_VehicleOrigin, 
SensorMount_offset_z_relative_to_VehicleOrigin] 

as the inputs and outputs the vehicle pose in ENU coordinates as 
[X_vehicle_ENU, Y_vehicle_ENU, Z_vehicle_ENU, ROLL_vehicle_ENU, 
PITCH_vehicle_ENU, YAW_vehicle_ENU]

**METHOD:** 

Step 1: Find the YAW of the vehicle relative to ENU coordinates
(YAW_vehicle_ENU)

1. - Find the unit vector from GPSLeft_ENU --> GPSRight_ENU

2.  - Find the orthogonal by rotating the unit vector by -90 degrees via 
a 3x3 matrix multiplication to determine vehicle orientation in Z 
axis of ENU coordinates 

3.  - The YAW is calculated by finding the arc tangent of the orthogonal 
of the unit vector. The 4-quadrant output is returned as an angle 
in radians using atan2.

Step 2: Find the ROLL of the vehicle relative to ENU coordinates
(ROLL_vehicle_ENU)

1. - The vehicle is rotated based on the YAW_vehicle_ENU found in the
previous step. The vehicle is rotated in a way that the YAW is zero.

2. - The vehicle is rotated based on the PITCH_vehicle_ENU given in the
input. The vehicle is rotated in a way that the PITCH is zero.

3. - Find the dot product of horizontal unit vector when YAW, PITCH, and 
ROLL are zero and the unit vector when YAW and PITCH are zero from 
left to right, and the corresponding norms are calculated to find 
the magnitude of the ROLL of the vehicle. acos function is used to
find the angle between the two unit vectors.

4. - The direction of the ROLL is obtained by calculating the cross
product of the two unit vectors mentioned above (horizontal unit 
vector when YAW, PITCH, and ROLL are zero and the unit vector when 
YAW and PITCH are zero from left to right). The sign of the
X-coordinate of the cross product determines the direction of the
ROLL.

Step 3: Find the Vehicle Origin, in ENU coordinates (vehicleOrigin_ENU)
based on YAW.

1. - Find the mid-point of the GPSLeft_ENU and GPSRight_ENU to find the
position (origin) of the sensor mount. 

2. - Translate the mid-point of the GPSLeft_ENU and GPSRight_ENU to 
vehicle's origin by assuming the "YAW" of the sensor mount relative 
to the vehicle as "zero", and assuming that the center of the 
vehicle is "ahead" in the vehicle coordinates of the sensor mount 
by a known distance. The projection from sensor mount to vehicle's 
origin is [+X_SensorMount_center, 0, -Z_SensorMount_center] 
assuming that the sensor mount is [-X_SensorMount_center, 0, 
+Z_SensorMount_center] relative to the vehicle's origin.

Step 4: Find the Vehicle Pose in ENU coordinates

* vehiclePose_ENU = [VehicleOrigin_ENU, ROLL_vehicle_ENU, PITCH_vehicle_ENU, YAW_vehicle_ENU]. 


**FORMAT:**

```MATLAB
vehiclePose_ENU = fcn_Transform_findVehiclePoseinENU(GPSLeft_ENU, GPSRight_ENU, PITCH_vehicle_ENU, SensorMount_offset_relative_to_VehicleOrigin, fig_num)
```
**INPUTS:**

GPSLeft_ENU: the center of the left GPS Antenna [xLeft, yLeft, 
zLeft] in meters

GPSRight_ENU: the center of the right GPS Antenna [xRight, yRight, 
zRight] in meters

PITCH_vehicle_ENU: the PITCH of the vehicle in ENU coordinates in
degrees

SensorMount_offset_relative_to_VehicleOrigin: the position of the
sensor mount relative to the vehicle origin [-X_SensorMount_center,
0, +Z_SensorMount_center]


(OPTIONAL INPUTS)   

fig_num: The figure is plotted if fig_num is entered as the input.

**OUTPUTS:**

vehiclePose_ENU: the position of the vehicle's origin and
orientation of the vehicle in ENU coordinates as [X_vehicle_ENU, 
Y_vehicle_ENU, Z_vehicle_ENU, ROLL_vehicle_ENU, PITCH_vehicle_ENU, 
YAW_vehicle_ENU]

**Dependencies:**

No dependencies 

**Examples:**

See the script: script_test_fcn_Transform_findVehiclePoseinENU.m for a full test suite.

<a href="#featureextraction_datatransforms_transformclasslibrary">Back to top</a>

***

#### **fcn_Transform_SensorCoordToENU**

This function takes vehicle parameters, sensor pose parameters,
sensor_or_vehicle string, vehiclePose_ENU as the inputs and outputs a 
ENU point's reading (sensorReading_SensorCoord) in ENU coordinates

**METHOD:** 

Step 1: Assign the parameters to the vehicle and the sensors

Step 2: If there are any perturbations in the position or orientation of
the sensors, they are added to the sensors before moving them to
the correct locations

Step 3: Set the pose of the vehicle based on the vehiclePose_ENU
(input)

Step 4: Find the transform matrices of the sensors and vehicle based on
sensor_or_vehicle string by multiplying the translation and 
rotation transform matrices of sensor and vehicle

Step 5: Find the transformed Point by multiplying the transformation
matrices
 
**FORMAT:**

```MATLAB
transformed_SensorCoord_in_ENU = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, sensor_or_vehicle, 
                                            vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num)
```

**INPUTS:**

vehicleParameters: The structure of all vehicle parameters

This structure includes dimensions of the vehicle and sensors, 
and the offsets of all the points to locate the origin. 


sensorPoseParameters: The structure of all sensor pose parameters

This structure includes the position and orientation of the 
sensors relative to the vehicle and the rear GPS Hemisphere sensor platform


sensor_or_vehicle: transform matrix of this "sensor_or_vehicle" is
generated to transform the "sensor_or_vehicle" 
coordinates to ENU and vice versa      


vehiclePose_ENU: [x,y,z,roll,pitch,yaw]

position of the vehicle:
<ul>
x: translates the vehicle in the x direction relative to
ENU coordinates

y: translates the vehicle in the y direction relative to
ENU coordinates

z: translates the vehicle in the z direction relative to
ENU coordinates
</ul>

orientation of the vehicle - Follows ISO convention

<ul>
roll: rotates the vehicle about its x-axis relative to ENU
coordinates (the vehicle's orientation changes relative to
the Earth's surface)

pitch: rotates the vehicle about its y-axis relative to ENU
coordinates (the vehicle's orientation changes relative to
the Earth's surface)

yaw: rotates the vehicle about its z-axis relative to ENU
coordinates (the vehicle's orientation changes relative to
the Earth's surface)
</ul>

sensorReading_SensorCoord: Sensor reading in sensor coordinates.

(OPTIONAL INPUTS)

perturbation_sensor: [x_Perturbation,y_Perturbation,z_Perturbation,
roll_Perturbation,pitch_Perturbation,yaw_Perturbation]

Perturbation in the position of the vehicle:

<ul>
x_Perturbation: This is the perturbation of the sensor in the x
direction, in cm, relative to the sensor platform coordinates.

y_Perturbation: This is the perturbation of the sensor in the y
direction, in cm, relative to the sensor platform coordinates.

z_Perturbation: This is the perturbation of the sensor in the z
direction, in cm, relative to the sensor platform coordinates.
</ul>

Perturbation IN THE orientation of the vehicle - Follows ISO 
convention

<ul>
roll_Perturbation: The perturbation of the sensor orientation 
about its x-axis 

pitch_Perturbation: The perturbation of the sensor orientation 
about its y-axis 

yaw_Perturbation: The perturbation of the sensor orientation 
about its z-axis 
</ul>

fig_num: The figure is plotted if fig_num is entered as the input. 

**OUTPUTS:**

transformed_SensorCoord_in_ENU: the sensor reading from sensor 
coordinates is transformed into the ENU coordinates. 
     
**Dependencies:**

No dependencies

**Examples:**

See the script: script_test_fcn_Transform_SensorCoordToENU
for a full test suite.

<a href="#featureextraction_datatransforms_transformclasslibrary">Back to top</a>

***

#### **fcn_Transform_ENUToSensorCoord**

This function takes vehicle parameters, sensor pose parameters,
sensor_or_vehicle string, vehiclePose_ENU as the inputs and outputs a 
ENU point's reading (sensorReading_ENU) in sensor coordinates

**METHOD:** 

Step 1: Assign the parameters to the vehicle and the sensors

Step 2: If there are any perturbations in the position or orientation of
the sensors, they are added to the sensors before moving them to
the correct locations

Step 3: Set the pose of the vehicle based on the vehiclePose_ENU
(input)

Step 4: Find the transform matrices of the sensors and vehicle based on
sensor_or_vehicle string by multiplying the translation and 
rotation transform matrices of sensor and vehicle

Step 5: Find the transformed Point by multiplying the transformation
matrices
 
**FORMAT:**

```MATLAB
transformed_ENUPoint_in_SensorCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, sensor_or_vehicle, 
                                            vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num)
```

**INPUTS:**

vehicleParameters: The structure of all vehicle parameters

This structure includes dimensions of the vehicle and sensors, 
and the offsets of all the points to locate the origin. 


sensorPoseParameters: The structure of all sensor pose parameters

This structure includes the position and orientation of the 
sensors relative to the vehicle and the rear GPS Hemisphere sensor platform


sensor_or_vehicle: transform matrix of this "sensor_or_vehicle" is
generated to transform the "sensor_or_vehicle" 
coordinates to ENU and vice versa      


vehiclePose_ENU: [x,y,z,roll,pitch,yaw]

position of the vehicle:
<ul>
x: translates the vehicle in the x direction relative to
ENU coordinates

y: translates the vehicle in the y direction relative to
ENU coordinates

z: translates the vehicle in the z direction relative to
ENU coordinates
</ul>

orientation of the vehicle - Follows ISO convention

<ul>
roll: rotates the vehicle about its x-axis relative to ENU
coordinates (the vehicle's orientation changes relative to
the Earth's surface)

pitch: rotates the vehicle about its y-axis relative to ENU
coordinates (the vehicle's orientation changes relative to
the Earth's surface)

yaw: rotates the vehicle about its z-axis relative to ENU
coordinates (the vehicle's orientation changes relative to
the Earth's surface)
</ul>

sensorReading_SensorCoord: Sensor reading in sensor coordinates.

(OPTIONAL INPUTS)

perturbation_sensor: [x_Perturbation,y_Perturbation,z_Perturbation,
roll_Perturbation,pitch_Perturbation,yaw_Perturbation]

Perturbation in the position of the vehicle:

<ul>
x_Perturbation: This is the perturbation of the sensor in the x
direction, in cm, relative to the sensor platform coordinates.

y_Perturbation: This is the perturbation of the sensor in the y
direction, in cm, relative to the sensor platform coordinates.

z_Perturbation: This is the perturbation of the sensor in the z
direction, in cm, relative to the sensor platform coordinates.
</ul>

Perturbation IN THE orientation of the vehicle - Follows ISO 
convention

<ul>
roll_Perturbation: The perturbation of the sensor orientation 
about its x-axis 

pitch_Perturbation: The perturbation of the sensor orientation 
about its y-axis 

yaw_Perturbation: The perturbation of the sensor orientation 
about its z-axis 
</ul>

fig_num: The figure is plotted if fig_num is entered as the input. 

**OUTPUTS:**
     
transformed_ENUPoint_in_SensorCoord: the sensor reading from ENU
coordinates is transformed into the corresponding sensor
coordinates. 
     
**Dependencies:**

No dependencies

**Examples:**

See the script: script_test_fcn_Transform_ENUToSensorCoord
for a full test suite.

<a href="#featureextraction_datatransforms_transformclasslibrary">Back to top</a>

***

<!-- USAGE EXAMPLES -->
## Usage
<!-- Use this space to show useful examples of how a project can be used.
Additional screenshots, code examples and demos work well in this space. You may
also link to more resources. -->

### General Usage

Each of the functions has an associated test script, using the convention

```sh
script_test_fcn_fcnname
```

where fcnname is the function name as listed above.

As well, each of the functions includes a well-documented header that explains inputs and outputs. These are supported by MATLAB's help style so that one can type:

```sh
help fcn_fcnname
```

for any function to view function details.

<a href="#featureextraction_datatransforms_transformclasslibrary">Back to top</a>

***
<!-- LICENSE -->
## License

Distributed under the MIT License. See `LICENSE` for more information.

<a href="#featureextraction_datatransforms_transformclasslibrary">Back to top</a>

***

## Major Release Versions

This code is still in development (alpha testing)

<a href="#featureextraction_datatransforms_transformclasslibrary">Back to top</a>

***

<!-- CONTACT -->
## Contact

Sean Brennan - sbrennan@psu.edu

Project Link: [https://github.com/ivsg-psu/FeatureExtraction_DataTransforms_TransformClassLibrary](https://github.com/ivsg-psu/FeatureExtraction_DataTransforms_TransformClassLibrary)

<a href="#featureextraction_datatransforms_transformclasslibrary">Back to top</a>

***

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
