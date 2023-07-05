
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
          <li><a href="#fcn_transform_determinesensor">fcn_Transform_determineSensor - Used for determining snesor type.</li>
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

1. Make sure to run MATLAB 2020b or higher.
2. Clone the repo

   ```sh
   git clone https://github.com/ivsg-psu/FeatureExtraction_DataTransforms_TransformClassLibrary
   ```
3. Confirm it works! Run the test scripts for each function. If the code works, the scripts should run without errors.

<a href="#featureextraction_datatransforms_transformclasslibrary">Back to top</a>

***

<!-- STRUCTURE OF THE REPO -->
### Directories

The following are the top level directories within the repository:
<ul>
 <li>Functions folder: Contains all functions and their test scripts.</li>
 <li>Scripts for development folder: Contains scripts used to develop the functions in this repository.</li>
 <li>Images folder: Contains images used in the README file.</li>
</ul>

<a href="#featureextraction_datatransforms_transformclasslibrary">Back to top</a>

***

### Dependencies

No dependencies 

<a href="#featureextraction_datatransforms_transformclasslibrary">Back to top</a>

***

<!-- FUNCTION DEFINITIONS -->
## Functions
#### **fcn_transform_predictWheelVelocity**
Usage: </br>

    This function predicts the velocity of the two rear wheels of the vehicle.

Inputs: </br>

    pos_rear_left: [x,y,z] position of rear left tire in meters. Expected value is [0, d, 0], where d is the distance from middle of rear vehicle axle to middle of wheel.

    pos_rear_right: [x,y,z] position of rear right tire in meters. Expected value is [0, - d, 0], where d is the distance from middle of rear vehicle axle to the middle of wheel.

    chassis_w: angular velocity of vehicle chassis in rad/s; vector; expected input is an array, [w_x, w_y,w_z], where w_x, w_y, and w_z are the components of the chassis' angular velocity.

    chassis_v: linear velocity of vehicle in m/s; vector; expected input is an array, [v_x, v_y, v_z], where v_x, v_y, v_z are components of the chassis' linear velocity.
            
Outputs:

    wheel_v_rear_left: velocity of rear left wheel in m/s; vector, [V_x, V_y, V_z], where V_x, V_y, V_z are the components of the linear velocity of the rear left wheel. 

    wheel_v_rear_right: velocity of rear right wheel in m/s; vector, [V_x, V_y, V_z], where V_x, V_y, V_z are the components of the linear velocity of the rear left wheel. 
      
Dependencies:

      No dependencies 

Examples:

    See the script: script_test_fcn_transform_predictWheelVelocity
    for a full test suite.

#### **fcn_transform_encoderCounts**
Usage: </br>

    This function calculates the number of encoder coutns for each wheel.

Inputs: </br>

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
     
Outputs:
     
      discrete_encoder_count_rear_left = An array of the calculated
                                         discrete encoder counts for the
                                         rear left wheel encoder. [counts]

      discrete_encoder_count_rear_right = An array of the calculated
                                         discrete encoder counts for the
                                         rear right wheel encoder. [counts]
      
Dependencies:

     No dependencies 

Examples:

    See the script: script_test_fcn_transform_encoderCounts
    for a full test suite.


#### **fcn_Transform_determineSensor**
Usage: </br>

    This function determines the sensor based on user inputs. 

INPUTS:

     type_of_sensor: The name of the sensor


Outputs:

     sensor: a string listing the data type, one of:
            'sick', 'velodyne', 'rightGPS', 'leftGPS', 'sensorplatform',
            'vehicle', 'all'.

     Note: If the sensor is not recognized, it lists 'other'.

Dependencies:

     No dependencies 

Examples:

    See the script: script_test_fcn_Transform_determineSensor
    for a full test suite.


#### **fcn_Transform_SensorCoordToENU**
Usage: </br>

    This function takes a point in sensor coordinates (sensorReading_SensorCoord),vehicle pose in ENU coordinates, and the sensor string as the inputs and outputs the reading (sensorReading_SensorCoord) in ENU coordinates.
 
Inputs:
     
     sensorReading_SensorCoord: Sensor reading in sensor coordinates.
      
     vehiclePose_ENU: [x,y,z,roll,pitch,yaw]

       position of the vehicle:

          x: translates the vehicle in the x direction relative to
          ENU coordinates
          y: translates the vehicle in the y direction relative to
          ENU coordinates
          z: translates the vehicle in the z direction relative to
          ENU coordinates

       orientation of the vehicle - Follows ISO convention

         roll: rotates the vehicle about its x-axis relative to ENU
         coordinates (the vehicle's orientation changes relative to
         the Earth's surface)

         pitch: rotates the vehicle about its y-axis relative to ENU
         coordinates (the vehicle's orientation changes relative to
         the Earth's surface)

         yaw: rotates the vehicle about its z-axis relative to ENU
         coordinates (the vehicle's orientation changes relative to
         the Earth's surface)
     
      sensor: this "sensor" coordinates are transformed into ENU 
      coordinates
            
Outputs:
     
     transformed_SensorCoord_in_ENU: the sensor reading from sensor
     coordinates is transformed into the ENU coordinates
     
Dependencies:

     No dependencies

EXAMPLES:

    See the script: script_test_fcn_Transform_SensorCoordToENU
    for a full test suite.


#### **fcn_Transform_ENUToSensorCoord**
Usage: </br>

    This function takes an ENU point (sensor reading in ENU coordinates), vehicle pose in ENU coordinates, and the sensor string as the inputs and outputs the ENU point's reading (sensorReading_ENU) in sensor coordinates.

Inputs:
     
     sensorReading_ENU: Sensor reading in ENU coordinates.
      
     vehiclePose_ENU: [x,y,z,roll,pitch,yaw]

       position of the vehicle:

          x: translates the vehicle in the x direction relative to
          ENU coordinates
          y: translates the vehicle in the y direction relative to
          ENU coordinates
          z: translates the vehicle in the z direction relative to
          ENU coordinates

       orientation of the vehicle - Follows ISO convention

         roll: rotates the vehicle about its x-axis relative to ENU
         coordinates (the vehicle's orientation changes relative to
         the Earth's surface)

         pitch: rotates the vehicle about its y-axis relative to ENU
         coordinates (the vehicle's orientation changes relative to
         the Earth's surface)

         yaw: rotates the vehicle about its z-axis relative to ENU
         coordinates (the vehicle's orientation changes relative to
         the Earth's surface)
     
     sensor: ENU coordinates are transformed to this "sensor"
             coordinates
            
Outputs:
     
     transformed_ENUPoint_in_SensorCoord: the sensor reading from ENU
     coordinates is transformed into the corresponding sensor
     coordinates. 

Dependencies:

     No dependencies

Examples:

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

## Major release versions

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
