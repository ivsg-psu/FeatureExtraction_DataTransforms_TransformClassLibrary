% script_test_fcn_Transform_SensorCoordToENU.m
% tests fcn_Transform_SensorCoordToENU.m

% Revision history
% 2023_07_02 - Aneesh Batchu
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

%% Vehicle Coordinates - Case 1

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. 
sensorReading_SensorCoord = [-5, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,0,0,0,0,0]; 

% The vehicle is moved 5 units in the X - direction relative to ENU
% coordinates. 
% 
% In "fcn_Transform_ENUToSensorCoord" cases, the sensorReading_ENU was [0,
% 0, 0]. The transformed point was [-5, 0, 0]. 
% In this case, if the sensorReading_SensorCoord = [-5, 0, 0] the expected
% answer of the transformed ENU point from vehicle coordinates will be
% [0, 0, 0]. 

% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases. 

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'vehicleCoord';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];
 
assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 2

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [0, 4, 0];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'vehicle';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];
 
assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 3

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [0, 0, -2];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'vehicle_coord';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];
 
assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 4

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. 
sensorReading_SensorCoord = [-4, 0, 0];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'vehicle coordinates';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [1, 0, 0];
 
assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 5

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. 
sensorReading_SensorCoord = [0, 4, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,0,0,0,0,90]; 

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'vehicle_Coord';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [1, 0, 0];
 
assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 6

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. 
sensorReading_SensorCoord = [0, 0, -4];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,0,0,0,90,0]; 

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'Coordinates_vehicle';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [1, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 7

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. 
sensorReading_SensorCoord = [-4, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,0,0,90,0,0]; 

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'vehicle';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [1, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 8 - vehilclePose_ENU with roll, pitch and yaw = 0.

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [5, -7, -3];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-5,7,3,0,0,0];

% The vehicle is moved -5 units in the X direction, 7 units in the 
% Y direction, and 3 units in the Z direction relative to ENU coordinates.
% coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases. 


% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'vehicle';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];
 
assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 9 - vehilclePose_ENU with yaw

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [-5, 4, 6];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [4,5,-6,0,0,90];

% The vehicle is moved 4 units in the X direction, 5 units in the 
% Y direction, and -6 units in the Z direction relative to ENU coordinates.
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


% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'vehicle';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 10 - vehilclePose_ENU with pitch

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [-6, -5, -4];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [4,5,-6,0,90,0];

% The vehicle is moved 4 units in the X direction, 5 units in the 
% Y direction, and -6 units in the Z direction relative to ENU coordinates.
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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'vehicle';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 11 - vehilclePose_ENU with position and roll 

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [-4, -6, -5];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [4,5,-6,90,0,0];

% The vehicle is moved 4 units in the X direction, 5 units in the 
% Y direction, and -6 units in the Z direction relative to ENU coordinates.
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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'vehicle';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 12 - vehilclePose_ENU with position, pitch and yaw

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [5, -9, 1];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [1,-9,5,0,90,180];

% The vehicle is moved 1 units in the X direction, -9 units in the 
% Y direction, and 5 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 0, 90, and 180, respectively
% relative to ENU coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases. 

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'vehicle';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 13 - vehilclePose_ENU with position, roll and pitch

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [1, -5, 9];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [1,-9,5,90,180,0];

% The vehicle is moved 1 units in the X direction, -9 units in the 
% Y direction, and 5 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 0, 90, and 180, respectively
% relative to ENU coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases. 

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'vehicle_coordinates';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 14 - vehilclePose_ENU with position, roll and yaw

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [9, -1, 5];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [1,-9,5,180,0,90];

% The vehicle is moved 1 units in the X direction, -9 units in the 
% Y direction, and 5 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 180, 0, and 90, respectively
% relative to ENU coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases. 

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'vehicle_coordinates';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 15 - vehilclePose_ENU with position, roll, pitch and yaw

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.

% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [5, 1, 9];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [1,-9,5,270,90,180];

% The vehicle is moved 1 units in the X direction, -9 units in the 
% Y direction, and 5 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 270, 90, and 180, respectively
% relative to ENU coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases. 

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'vehicle_coordinates';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 16 - vehilclePose_ENU with position, and roll, pitch and yaw greater than 360 degrees

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [5, 1, 9];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [1,-9,5,360+270,720+90,1080+180];

% The vehicle is moved 1 units in the X direction, -9 units in the 
% Y direction, and 5 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 360+270, 720+90, and 1080+180, respectively
% relative to ENU coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases. 

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'vehicle_coordinates';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Vehicle Coordinates - Case 17 - vehilclePose_ENU with position, and negative roll, pitch and yaw

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [-5, 1, -9];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [1,-9,5,-270,-90,-180];

% The vehicle is moved 1 units in the X direction, -9 units in the 
% Y direction, and 5 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 270, 90, and 180, respectively
% relative to ENU coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases. 

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'vehicle_coordinates';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 18 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. 
sensorReading_SensorCoord = [0, -3, 2];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'vehicleCoord';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [5, -3, 2];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 19 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. 
sensorReading_SensorCoord = [-7, -10,-5];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,8,0,0,0,0]; 

% The vehicle is moved 8 units in the Y - direction relative to ENU
% coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases. 

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'vehicle Coord';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [-7, -2, -5];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 20 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. 
sensorReading_SensorCoord = [4, 6, 2];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,-6,0,0,0]; 

% The vehicle is moved -6 units in the Z - direction relative to ENU
% coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases. 

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'vehicle Coord';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [4, 6, -4];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 21 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. 
sensorReading_SensorCoord = [6, -8, 7];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-4,7,2,0,0,0]; 

% The vehicle is moved -4 units in the X direction, 7 units in the 
% Y direction, and 2 units in the Z direction relative to ENU coordinates.
% coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases. 

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'vehicle Coord';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [2, -1, 9];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 22 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. 
sensorReading_SensorCoord = [-8, -6, 7];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-4,7,2,0,0,90]; 

% The vehicle is moved -4 units in the X direction, 7 units in the 
% Y direction, and 2 units in the Z direction relative to ENU coordinates.
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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'vehicle Coord';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [2, -1, 9];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 23 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. 
sensorReading_SensorCoord = [-7, -8, 6];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-4,7,2,0,90,0]; 

% The vehicle is moved -4 units in the X direction, 7 units in the 
% Y direction, and 2 units in the Z direction relative to ENU coordinates.
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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'vehicle Coord';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [2, -1, 9];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 24 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. 
sensorReading_SensorCoord = [6, -7, -8];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-4,7,2,90,0,0]; 

% The vehicle is moved -4 units in the X direction, 7 units in the 
% Y direction, and 2 units in the Z direction relative to ENU coordinates.
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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'vehicle Coord';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [2, -1, 9];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 25 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. 
sensorReading_SensorCoord = [-8, 7, 6];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-4,7,2,90,180,270]; 

% The vehicle is moved -4 units in the X direction, 7 units in the 
% Y direction, and 2 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 90, 180, and 270, respectively
% relative to ENU coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases. 

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'vehicle Coord';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [2, -1, 9];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 26 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. 
sensorReading_SensorCoord = [6, -8, 7];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-4,7,2,0,0,0]; 

% The vehicle is moved -4 units in the X direction, 7 units in the 
% Y direction, and 2 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 360, 360, and 360, respectively
% relative to ENU coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases. 

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'vehicle Coord';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [2, -1, 9];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));


%% The following cases were written to test the accuracy of sensor platform coordinates 

% when a point(sensorReading) in ENU coordinates is transformed to 
% sensor platform coordinates


%% Sensor Platform - Case 1

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sensor platform coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [1, 0, -1.6];

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


% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'sensor___platform';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));


%% Sensor Platform - Case 2

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sensor platform coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [-4, 0, -1.6];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'sensor_platform';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sensor Platform - Case 3

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sensor platform coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [1, 4, -1.6];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'sensor_platform';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sensor Platform - Case 4

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sensor platform coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [1, 0, -3.6];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'sensor_platform';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sensor Platform - Case 5

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sensor platform coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [9, -1, 4.4];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'sensor_platform';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sensor Platform - Case 6

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sensor platform coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [9, 4, -2.6];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'sensor_platform';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sensor Platform - Case 7

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sensor platform coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [-2.4, -7, 0];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'sensor_platform';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sensor Platform - Case 8

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sensor platform coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [10, 4.6, 7];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'sensor_platform';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sensor Platform - Case 9

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sensor platform coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [1.4, -2, -4];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [3,4,-3,180,270,90];

% The vehicle is moved 3 units in the X direction, 4 units in the 
% Y direction, and -3 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 180, 270, and 90, respectively
% relative to ENU coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases. 

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'sensor_platform';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));


%% The following cases were written to test the accuracy of sick lidar coordinates 

% when a point(sensorReading) in ENU coordinates is transformed to 
% sick lidar coordinates


%% Sick Lidar - Case 1

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sick lidar coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [1.5, 0, 1.4];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'sickLidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sick Lidar - Case 2

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sick lidar coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [-3.5, 0, 1.4];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'sickLidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sick Lidar - Case 3

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sick lidar coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [1.5, 4, 1.4];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'sick_lidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sick Lidar - Case 4

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sick lidar coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [1.5, 0, -0.6];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'sick_lidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Sick Lidar - Case 5

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sick lidar coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [9.5, -1, 7.4];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'sick_lidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sick Lidar - Case 6

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sick lidar coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [9, 3.5, 0.4];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'sick_lidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));


%% Sick Lidar - Case 7

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sick lidar coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [-5.4, -7, 0.5];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'sick_lidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));


%% Sick Lidar - Case 8

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sick lidar coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [10.5, 1.6, 7];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'sick_lidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));


%% Sick Lidar - Case 9

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sick lidar coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [4.4, -1.5, -4];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [3,4,-3,180,270,90];

% The vehicle is moved 3 units in the X direction, 4 units in the 
% Y direction, and -3 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 180, 270, and 90, respectively
% relative to ENU coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases.

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'sick_lidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));


%% The following cases were written to test the accuracy of velodyne lidar coordinates 

% when a point(sensorReading) in ENU coordinates is transformed to 
% velodyne lidar coordinates


%% Velodyne Lidar - Case 1

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into velodyne lidar coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [0, -0.4425, -2.7936];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'velodyne';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Velodyne Lidar - Case 2

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into velodyne lidar coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [-5, -0.4425, -2.7936];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'velodyne_lidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Velodyne Lidar - Case 3

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into velodyne lidar coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [0, 3.5575, -2.7936];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'velodyneLidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Velodyne Lidar - Case 4

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into velodyne lidar coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [0, -0.4425, -4.7936];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'velodynelidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));


%% The following cases were written to test the accuracy of left GPS coordinates 

% when a point(sensorReading) in ENU coordinates is transformed to 
% left GPS coordinates


%% Left GPS - Case 1

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into left GPS coordinates.

% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [0.7, -0.762, -2.1];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'leftGPS';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Left GPS - Case 2

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into left GPS coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [-4.3, -0.762, -2.1];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'left GPS';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Left GPS - Case 3

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into left GPS coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [0.7, 3.238, -2.1];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'left_GPS';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Left GPS - Case 4

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into left GPS coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [0.7, -0.762, -4.1];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'leftgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Left GPS - Case 5

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into left GPS coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [8.7, -1.762, 3.9];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'leftgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Left GPS - Case 6

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into left GPS coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [8.238, 4.3, -3.1];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'leftgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Left GPS - Case 7

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into left GPS coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [-1.9, -7.762, -0.3];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'leftgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Left GPS - Case 8

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into left GPS coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [9.7, 5.1, 6.238];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'leftgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Left GPS - Case 9

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into left GPS coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [0.9, -2.3, -4.762];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'leftgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% The following cases were written to test the accuracy of right GPS coordinates 

% when a point(sensorReading) in ENU coordinates is transformed to 
% right GPS coordinates


%% right GPS - Case 1

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into right GPS coordinates.

% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [0.7, 0.762, -2.1];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'rightGPS';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% right GPS - Case 2

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into right GPS coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [-4.3, 0.762, -2.1];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'right GPS';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% right GPS - Case 3

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into right GPS coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [0.7, 4.762, -2.1];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'right_GPS';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% right GPS - Case 4

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into right GPS coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [0.7, 0.762, -4.1];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'rightgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% right GPS - Case 5

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into right GPS coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [8.7, -0.238, 3.9];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'rightgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% right GPS - Case 6

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into right GPS coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [9.762, 4.3, -3.1];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'rightgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% right GPS - Case 7

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into right GPS coordinates.

% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [-1.9, -6.238, -0.3];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'rightgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% right GPS - Case 8

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into right GPS coordinates.

% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [9.7, 5.1, 7.762];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'rightgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% right GPS - Case 9

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into right GPS coordinates.

% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [0.9, -2.3, -3.238];

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

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'rightgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Fail Conditions 

% input cannot be NaN

if 1 == 0

 % A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [NaN, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [3,4,-3,NaN,270,90];

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'rightgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0.9, -2.3, -3.238];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

end

%% 

% input cannot be inf

if 1 == 0

 % A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,4,-3,0,270,inf];

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'rightgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0.9, -2.3, -3.238];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

end

%% 

% input cannot be inf or NaN

if 1 == 0

 % A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [0, NaN, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [inf,4,-3,0,NaN,0];

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'rightgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0.9, -2.3, -3.238];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

end

%% 

% sensorReading_ENU must be 1 x 3

if 1 == 0

 % A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [3,4,-3,180,270,90];


% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'rightgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0.9, -2.3, -3.238];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

end

%% 

% sensorReading_ENU must be 1 x 6

if 1 == 0

 % A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [3,4,-3,180,270];


% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'rightgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0.9, -2.3, -3.238];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

end

%% 

% input arguments should not exceed 3

if 1 == 0

 % A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [3,4,-3,180,270];

SensorVals = [3, 4, 6];

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'rightgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord, SensorVals);
expected_transformed_ENUPoint_from_dashCoord = [0.9, -2.3, -3.238];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

end

%% 

% input arguments should not be less than 3

if 1 == 0

 % A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [0, 0, 0];

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'rightgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0.9, -2.3, -3.238];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

end