% script_test_fcn_Transform_SensorCoordToENU_GPSSparkFunLeftRear.m
% tests fcn_Transform_SensorCoordToENU.m
%
% This script uses fcn_Transform_SensorCoordToENU.m to transform the ENU
% coordinates to vehicle coordinates. 
%
% Revision history
% 2023_09_27 - Aneesh Batchu
% -- wrote the code originally

%% Set up the workspace
close all
clc

%% Check assertions for basic path operations and function testing
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                              _   _                 
%      /\                     | | (_)                
%     /  \   ___ ___  ___ _ __| |_ _  ___  _ __  ___ 
%    / /\ \ / __/ __|/ _ \ '__| __| |/ _ \| '_ \/ __|
%   / ____ \\__ \__ \  __/ |  | |_| | (_) | | | \__ \
%  /_/    \_\___/___/\___|_|   \__|_|\___/|_| |_|___/
%                                                    
%                                                    
% See: https://patorjk.com/software/taag/#p=display&f=Big&t=Assertions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Load the example data

run('Example_vehicleParameters_and_sensorPoseParameters_Struct.m')

%% Left GPS - Case 1

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.

% A point in left GPS coordinates.  
sensorReading_SensorCoord = [0.6027 -0.7295 -1.6382];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,0,0,0,0];

% The vehicle's position is at the origin and the orientation of the
% vehicle is also zero which implies -- roll, pitch and yaw are zero
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases.

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'GPS_SparkFun_LeftRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));

%% Left GPS - Case 2

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.


% A point in left GPS coordinates.  
sensorReading_SensorCoord = [-4.3973 -0.7295 -1.6382];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,0,0,0,0,0];

% The vehicle is moved 5 units in the X - direction relative to ENU
% coordinates.
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases.

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'GPS_SparkFun_LeftRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));

%% Left GPS - Case 3

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.


% A point in left GPS coordinates.  
sensorReading_SensorCoord = [0.6027  3.2705 -1.6382];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,-4,0,0,0,0];

% The vehicle is moved -4 units in the Y - direction relative to ENU
% coordinates.
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases.

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'GPS_SparkFun_LeftRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));

%% Left GPS - Case 4

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.


% A point in left GPS coordinates.  
sensorReading_SensorCoord = [0.6027 -0.7295 -3.6382];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,2,0,0,0];

% The vehicle is moved 2 units in the Z - direction relative to ENU
% coordinates.
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases.

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'GPS_SparkFun_LeftRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));

%% Left GPS - Case 5

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.


% A point in left GPS coordinates.  
sensorReading_SensorCoord = [8.6027 -1.7295 4.3618];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-8,1,-6,0,0,0];

% The vehicle is moved -8 units in the X direction, 1 units in the 
% Y direction, and -6 units in the Z direction relative to ENU coordinates.
% coordinates.
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases.

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'GPS_SparkFun_LeftRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));

%% Left GPS - Case 6

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.


% A point in left GPS coordinates.  
sensorReading_SensorCoord = [9.6027 4.2705 -2.6382];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,-9,1,0,0,90];

% The vehicle is moved 5 units in the X direction, -9 units in the 
% Y direction, and 1 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 0, 0, and 90, respectively
% relative to ENU coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases.

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'GPS_SparkFun_LeftRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));

%% Left GPS - Case 7

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.


% A point in left GPS coordinates.  
sensorReading_SensorCoord = [-3.3973 -7.7295 -2.6382];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [1,7,-4,0,90,0];

% The vehicle is moved 1 units in the X direction, 7 units in the 
% Y direction, and -4 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 0, 90, and 0, respectively
% relative to ENU coordinates.
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases.

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'GPS_SparkFun_LeftRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));

%% Left GPS - Case 8

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.


% A point in left GPS coordinates.  
sensorReading_SensorCoord = [9.6027 -3.7295 -8.6382];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-9,-7,3,90,0,0];

% The vehicle is moved -9 units in the X direction, -7 units in the 
% Y direction, and 3 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 90, 0, and 0, respectively
% relative to ENU coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases.

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'GPS_SparkFun_LeftRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));

%% Left GPS - Case 9

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.


% A point in left GPS coordinates.  
sensorReading_SensorCoord = [3.6027 -3.7295 -5.6382];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [3,4,-3,180,270,90];

% The vehicle is moved 3 units in the X direction, 4 units in the 
% Y direction, and -3 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 180, 270, and 90, respectively
% relative to ENU coordinate
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases.

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'GPS_SparkFun_LeftRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));

%% Left GPS - Case 10

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.

% A point in left GPS coordinates. 
sensorReading_SensorCoord = [6.6027 -4.7295 7.3618];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,0,0,0,0];

% The vehicle's position is at the origin and the orientation of the
% vehicle is also zero which implies -- roll, pitch and yaw are zero
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases.

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'GPS_SparkFun_LeftRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [6, -4, 9];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));

%% Left GPS - Case 11

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.


% A point in left GPS coordinates. 
sensorReading_SensorCoord = [7.6027 3.2705 -2.6382];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,0,0,0,0,0];

% The vehicle is moved 5 units in the X - direction relative to ENU
% coordinates.
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases.

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'GPS_SparkFun_LeftRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [12, 4, -1];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));

%% Left GPS - Case 12

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.


% A point in left GPS coordinates. 
sensorReading_SensorCoord = [-7.3973 12.2705 1.3618];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,-4,0,0,0,0];

% The vehicle is moved -4 units in the Y - direction relative to ENU
% coordinates.
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases.

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'GPS_SparkFun_LeftRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [-8, 9, 3];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));

%% Left GPS - Case 13

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.


% A point in left GPS coordinates. 
sensorReading_SensorCoord = [6.6027 1.2705 5.3618];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,2,0,0,0];

% The vehicle is moved 2 units in the Z - direction relative to ENU
% coordinates.
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases.

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'GPS_SparkFun_LeftRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [6, 2, 9];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));

%% Left GPS - Case 14

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.


% A point in left GPS coordinates. 
sensorReading_SensorCoord = [19.6027 -6.7295 8.3618];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-8,1,-6,0,0,0];

% The vehicle is moved -8 units in the X direction, 1 units in the 
% Y direction, and -6 units in the Z direction relative to ENU coordinates.
% coordinates.
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases.

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'GPS_SparkFun_LeftRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [11, -5, 4];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));

%% Left GPS - Case 15

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.


% A point in left GPS coordinates. 
sensorReading_SensorCoord = [8.6027 -0.7295 6.3618];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,-9,1,0,0,90];

% The vehicle is moved 5 units in the X direction, -9 units in the 
% Y direction, and 1 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 0, 0, and 90, respectively
% relative to ENU coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases.

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'GPS_SparkFun_LeftRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [5, -1, 9];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));

%% Left GPS - Case 16

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.


% A point in left GPS coordinates. 
sensorReading_SensorCoord = [-11.3973 -5.7295 -11.6382];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [1,7,-4,0,90,0];

% The vehicle is moved 1 units in the X direction, 7 units in the 
% Y direction, and -4 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 0, 90, and 0, respectively
% relative to ENU coordinates.
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases.

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'GPS_SparkFun_LeftRear323';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [-9, 2, 8];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));

%% Left GPS - Case 17

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.


% A point in left GPS coordinates. 
sensorReading_SensorCoord = [15.6027 -2.7295 -5.6382];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-9,-7,3,90,0,0];

% The vehicle is moved -9 units in the X direction, -7 units in the 
% Y direction, and 3 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 90, 0, and 0, respectively
% relative to ENU coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases.

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'GPS_aw344SparkFun_LeftRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [6, -3, 1];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));

%% Left GPS - Case 18

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.


% A point in left GPS coordinates. 
sensorReading_SensorCoord = [7.6027 -5.7295 3.3618];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [3,4,-3,180,270,90];

% The vehicle is moved 3 units in the X direction, 4 units in the 
% Y direction, and -3 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 180, 270, and 90, respectively
% relative to ENU coordinate
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases.

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'GPS_SparkFun_LeftRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [-2, 9, 4];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));
