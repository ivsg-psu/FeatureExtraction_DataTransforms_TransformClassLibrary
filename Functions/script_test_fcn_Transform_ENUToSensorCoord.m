% script_test_fcn_Transform_ENUToSensorCoord.m
% tests fcn_Transform_ENUToSensorCoord.m

% Revision history
% 2023_06_29 - Aneesh Batchu
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
%% The cases from 1 - 17 are written to test the accuracy of vehicle coordinates 

% when a point(sensorReading) in ENU coordinates is transformed to 
% vehicle coordinates

% TO DO
%
% cases for different sensorReadings
% fail conditions for inputs like inf and NaN. 

%% Case 1

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,0,0,0,0,0]; 

% The vehicle is moved 5 units in the X - direction relative to ENU
% coordinates. Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [-5, 0, 0]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle';  

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, in_dashCoord);
expected_transformed_ENUPoint_in_dashCoord = [-5, 0, 0];
 
assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Case 2

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,-4,0,0,0,0];

% The vehicle is moved -4 units in the Y - direction relative to ENU
% coordinates. Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [0, 4, 0]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle';  

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, in_dashCoord);
expected_transformed_ENUPoint_in_dashCoord = [0, 4, 0];
 
assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Case 3

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,2,0,0,0];

% The vehicle is moved 2 units in the Z - direction relative to ENU
% coordinates. Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [0, 0, -2];

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle';  

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, in_dashCoord);
expected_transformed_ENUPoint_in_dashCoord = [0, 0, -2];
 
assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Case 4

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [1, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,0,0,0,0,0]; 

% The vehicle is moved 5 units in the X - direction relative to ENU
% coordinates. Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [-4, 0, 0]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle';  

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, in_dashCoord);
expected_transformed_ENUPoint_in_dashCoord = [-4, 0, 0];
 
assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Case 5

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [1, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,0,0,0,0,90]; 

% The vehicle is moved 5 units in the X - direction relative to ENU
% coordinates. Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [0, 4, 0]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle';  

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, in_dashCoord);
expected_transformed_ENUPoint_in_dashCoord = [0, 4, 0];
 
assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Case 6

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [1, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,0,0,0,90,0]; 

% The vehicle is moved 5 units in the X - direction relative to ENU
% coordinates. Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [0, 0, -4]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle';  

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, in_dashCoord);
expected_transformed_ENUPoint_in_dashCoord = [0, 0, -4];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Case 7

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [1, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,0,0,90,0,0]; 

% The vehicle is moved 5 units in the X - direction relative to ENU
% coordinates. Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [0, 0, -4]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle';  

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, in_dashCoord);
expected_transformed_ENUPoint_in_dashCoord = [-4, 0, 0];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Case 8

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-5,7,3,0,0,0];

% The vehicle is moved -5 units in the X direction, 7 units in the 
% Y direction, and 3 units in the Z direction relative to ENU coordinates.
% coordinates. Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [5, -7, -3];

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle';  

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, in_dashCoord);
expected_transformed_ENUPoint_in_dashCoord = [5, -7, -3];
 
assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Case 9

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [4,5,-6,0,0,90];

% The vehicle is moved 4 units in the X direction, 5 units in the 
% Y direction, and -6 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 0, 0, and 90, respectively
% relative to ENU coordinates. 
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [-5, 4, 6];

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle';  

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, in_dashCoord);
expected_transformed_ENUPoint_in_dashCoord = [-5, 4, 6];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Case 10

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [4,5,-6,0,90,0];

% The vehicle is moved 4 units in the X direction, 5 units in the 
% Y direction, and -6 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 0, 90, and 0, respectively
% relative to ENU coordinates. 
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [-6, -5, -4];

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle';  

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, in_dashCoord);
expected_transformed_ENUPoint_in_dashCoord = [-6, -5, -4];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Case 11

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [4,5,-6,90,0,0];

% The vehicle is moved 4 units in the X direction, 5 units in the 
% Y direction, and -6 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 90, 0, and 0, respectively
% relative to ENU coordinates. 
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [-4, 6, 5];

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle';  

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, in_dashCoord);
expected_transformed_ENUPoint_in_dashCoord = [-4, 6, 5];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Case 12

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [1,-9,5,0,60,90];

% The vehicle is moved 1 units in the X direction, -9 units in the 
% Y direction, and 5 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 0, 60, and 90, respectively
% relative to ENU coordinates. 
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [X, X, X];

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle';  

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, in_dashCoord);
% expected_transformed_ENUPoint_in_dashCoord = [-4, 6, 5];
% 
% assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Case 13

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [1,-9,5,20,0,45];

% The vehicle is moved 1 units in the X direction, -9 units in the 
% Y direction, and 5 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 20, 0, and 90, respectively
% relative to ENU coordinates. 
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [X, X, X];

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle';  

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, in_dashCoord);
% expected_transformed_ENUPoint_in_dashCoord = [-4, 6, 5];
% 
% assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Case 14

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [1,-9,5,45,30,0];

% The vehicle is moved 1 units in the X direction, -9 units in the 
% Y direction, and 5 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 45, 30, and 0, respectively
% relative to ENU coordinates. 
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [X, X, X];

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle';  

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, in_dashCoord);
% expected_transformed_ENUPoint_in_dashCoord = [-4, 6, 5];
% 
% assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Case 15

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,3,-2,36,24,99];

% The vehicle is moved 5 units in the X direction, 3 units in the 
% Y direction, and -2 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 36, 24, and 99, respectively
% relative to ENU coordinates. 
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [X, X, X];

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle';  

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, in_dashCoord);
% expected_transformed_ENUPoint_in_dashCoord = [-4, 6, 5];
% 
% assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Case 16

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,3,-2,720+36,360+24,360+99];

% The vehicle is moved 5 units in the X direction, 3 units in the 
% Y direction, and -2 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 720+36, 360+24, and 360+99, respectively
% relative to ENU coordinates. 
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [X, X, X];
% Note: The transformed_ENUPoint_in_dashCoord in case 15 and case 16 should
% be same

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle';  

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, in_dashCoord);
% expected_transformed_ENUPoint_in_dashCoord = [-4, 6, 5];
% 
% assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Case 17

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,3,-2,-36,-24,-99];

% The vehicle is moved 5 units in the X direction, 3 units in the 
% Y direction, and -2 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are -36, -24, and -99, respectively
% relative to ENU coordinates. 
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [X, X, X];


% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle';  

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, in_dashCoord);
% expected_transformed_ENUPoint_in_dashCoord = [-4, 6, 5];
% 
% assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Case 18 

% Fail Conditions


%% The cases from 17 - XX are written to test the accuracy of sensor platform coordinates 

% when a point(sensorReading) in ENU coordinates is transformed to 
% sensor platform coordinates


%% Case XX

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sensor platform coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,0,0,0,0];

% The vehicle's position is at the origin and the orientation of the
% vehicle is also zero which implies -- roll, pitch and yaw are zero
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in sensor platform coordinates will be [1, 0, -1.6]
% which is the -1*position of the sensor platform with respect to 
% vehicle coordinates.


% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'sensor___platform';  

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, in_dashCoord);
expected_transformed_ENUPoint_in_dashCoord = [1, 0, -1.6];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));


%% Case XX2

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sensor platform coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,0,0,0,0,0];

% The vehicle is moved 5 units in the X - direction relative to ENU
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in sensor platform coordinates will be [-4, 0, -1.6]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'sensor_platform';  

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, in_dashCoord);
expected_transformed_ENUPoint_in_dashCoord = [-4, 0, -1.6];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Case XX3

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sensor platform coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,-4,0,0,0,0];

% The vehicle is moved -4 units in the Y - direction relative to ENU
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in sensor platform coordinates will be [1, 4, -1.6]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'sensor_platform';  

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, in_dashCoord);
expected_transformed_ENUPoint_in_dashCoord = [1, 4, -1.6];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Case XX4

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sensor platform coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,2,0,0,0];

% The vehicle is moved 2 units in the Z - direction relative to ENU
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in sensor platform coordinates will be [1, 0, -3.6]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'sensor_platform';  

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, in_dashCoord);
expected_transformed_ENUPoint_in_dashCoord = [1, 0, -3.6];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Case XX5

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sensor platform coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,2,0,0,0];

% The vehicle is moved 2 units in the Z - direction relative to ENU
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in sensor platform coordinates will be [1, 0, -3.6]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'sensor_platform';  

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, in_dashCoord);
expected_transformed_ENUPoint_in_dashCoord = [1, 0, -3.6];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% The cases from XX - XX are written to test the accuracy of sick lidar coordinates 

% when a point(sensorReading) in ENU coordinates is transformed to 
% sick lidar coordinates


%% Case XX

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sick lidar coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,0,0,0,0];

% The vehicle's position is at the origin and the orientation of the
% vehicle is also zero which implies -- roll, pitch and yaw are zero
%
% Since we know the offset position of the sick lidar from the sensor 
% platform and how it is oriented, the expected answer of the transformed
% sensorReading_ENU in sick lidar coordinates will be [1.5, 0, 1.4]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'sickLidar';  

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, in_dashCoord);
expected_transformed_ENUPoint_in_dashCoord = [1.5, 0, 1.4];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% The cases from XX - XX are written to test the accuracy of velodyne lidar coordinates 

% when a point(sensorReading) in ENU coordinates is transformed to 
% velodyne lidar coordinates


%% Case XX

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into velodyne lidar coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,0,0,0,0];

% The vehicle's position is at the origin and the orientation of the
% vehicle is also zero which implies -- roll, pitch and yaw are zero
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in velodyne lidar coordinates will be [0, X, X]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'velodyne';  

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, in_dashCoord);
% expected_transformed_ENUPoint_in_dashCoord = [0, X, X];
% 
% assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% The cases from XX - XX are written to test the accuracy of left GPS coordinates 

% when a point(sensorReading) in ENU coordinates is transformed to 
% left GPS coordinates


%% Case XX

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into left GPS coordinates.

% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,0,0,0,0];

% The vehicle's position is at the origin and the orientation of the
% vehicle is also zero which implies -- roll, pitch and yaw are zero
%
% Since we know the offset position of the left GPS from the sensor 
% platform and how it is oriented, the expected answer of the transformed
% sensorReading_ENU in left GPS coordinates will be [0.7, -0.762, -2.1]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'leftGPS';  

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, in_dashCoord);
expected_transformed_ENUPoint_in_dashCoord = [0.7, -0.762, -2.1];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% The cases from XX - XX are written to test the accuracy of right GPS coordinates 

% when a point(sensorReading) in ENU coordinates is transformed to 
% right GPS coordinates


%% Case XX

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into right GPS coordinates.

% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,0,0,0,0];

% The vehicle's position is at the origin and the orientation of the
% vehicle is also zero which implies -- roll, pitch and yaw are zero
%
% Since we know the offset position of the right GPS from the sensor 
% platform and how it is oriented, the expected answer of the transformed
% sensorReading_ENU in right GPS coordinates will be [0.7, 0.762, -2.1]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'rightGPS';  

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, in_dashCoord);
expected_transformed_ENUPoint_in_dashCoord = [0.7, 0.762, -2.1];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

