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
%% The following cases were written to test the accuracy of vehicle coordinates 

% when a point(sensorReading_SensorCoord) in vehicle coordinates is 
% transformed to ENU coordinates

%% Vehicle Coordinates - Case 1

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicleCoord';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];
 
assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 2

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];
 
assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 3

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle_coord';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];
 
assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 4

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle coordinates';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [1, 0, 0];
 
assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 5

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle_Coord';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [1, 0, 0];
 
assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 6

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'Coordinates_vehicle';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [1, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 7

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [1, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 8 - vehilclePose_ENU with roll, pitch and yaw = 0.

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
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


% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];
 
assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 9 - vehilclePose_ENU with yaw

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 10 - vehilclePose_ENU with pitch

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 11 - vehilclePose_ENU with position and roll 

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 12 - vehilclePose_ENU with position, pitch and yaw

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 13 - vehilclePose_ENU with position, roll and pitch

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle_coordinates';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 14 - vehilclePose_ENU with position, roll and yaw

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle_coordinates';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 15 - vehilclePose_ENU with position, roll, pitch and yaw

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.

% A point in vehicle coordinates. 
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle_coordinates';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 16 - vehilclePose_ENU with position, and roll, pitch and yaw greater than 360 degrees

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle_coordinates';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Vehicle Coordinates - Case 17 - vehilclePose_ENU with position, and negative roll, pitch and yaw

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates.  
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle_coordinates';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 18 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicleCoord';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [5, -3, 2];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 19 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle Coord';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [-7, -2, -5];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 20 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle Coord';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [4, 6, -4];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 21 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle Coord';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [2, -1, 9];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 22 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle Coord';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [2, -1, 9];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 23 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.

% A point in vehicle coordinates. 
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle Coord';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [2, -1, 9];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 24 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle Coord';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [2, -1, 9];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 25 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle Coord';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [2, -1, 9];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 26 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle Coord';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [2, -1, 9];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));


%% The following cases were written to test the accuracy of sensor platform coordinates 

% when a point(sensorReading_SensorCoord) in sensor platform coordinates is 
% transformed to ENU coordinates


%% Sensor Platform - Case 1

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.


% A point in sensor platform coordinates. 
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


% Transforms this coordinates into ENU coordinates
from_dashCoord = 'sensor___platform';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));


%% Sensor Platform - Case 2

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.


% A point in sensor platform coordinates. 
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'sensor_platform';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sensor Platform - Case 3

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.


% A point in sensor platform coordinates. 
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'sensor_platform';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sensor Platform - Case 4

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.


% A point in sensor platform coordinates. 
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'sensor_platform';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sensor Platform - Case 5

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.


% A point in sensor platform coordinates. 
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'sensor_platform';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sensor Platform - Case 6

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.


% A point in sensor platform coordinates. 
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'sensor_platform';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sensor Platform - Case 7

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.


% A point in sensor platform coordinates. 
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'sensor_platform';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sensor Platform - Case 8

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.


% A point in sensor platform coordinates. 
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'sensor_platform';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sensor Platform - Case 9

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.


% A point in sensor platform coordinates. 
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'sensor_platform';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sensor Platform - Case 10

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.


% A point in sensor platform coordinates. 
sensorReading_SensorCoord = [6, 3, -3.6];

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
from_dashCoord = 'sensor___platform';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [5, 3, -2];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));


%% Sensor Platform - Case 11

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.


% A point in sensor platform coordinates. 
sensorReading_SensorCoord = [-3, -3, 4.4];

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
from_dashCoord = 'sensor_platform';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [1, -3, 6];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sensor Platform - Case 12

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.


% A point in sensor platform coordinates. 
sensorReading_SensorCoord = [-3, 11, -2.6];

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
from_dashCoord = 'sensor_platform';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [-4, 7, -1];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sensor Platform - Case 13

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.


% A point in sensor platform coordinates. 
sensorReading_SensorCoord = [8, 2, -7.6];

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
from_dashCoord = 'sensor_platform';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [7, 2, -4];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Sensor Platform - Case 14

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.


% A point in sensor platform coordinates. 
sensorReading_SensorCoord = [11, -8, 15.4];

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
from_dashCoord = 'sensor_platform';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [2, -7, 11];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sensor Platform - Case 15

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.

% A point in sensor platform coordinates. 
sensorReading_SensorCoord = [7, 3, 0.4];

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
from_dashCoord = 'sensor_platform';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [1, -2, 3];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Sensor Platform - Case 16

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.

% A point in sensor platform coordinates. 
sensorReading_SensorCoord = [-3.4, 1, -2];

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
from_dashCoord = 'sensor_platform';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [-2, 8, 1];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sensor Platform - Case 17

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.

% A point in sensor platform coordinates. 
sensorReading_SensorCoord = [15, 2.6, 8];

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
% sensorReading_ENU in sensor platform coordinates will be [15, 2.6, 8]

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'sensor_platform';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [5, 1, 2];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Sensor Platform - Case 18

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.

% A point in sensor platform coordinates. 
sensorReading_SensorCoord = [4.4, -1, -3];

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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'sensor_platform';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [1, 1, 3];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% The following cases were written to test the accuracy of sick lidar coordinates 

% when a point(sensorReading_SensorCoord) in sick lidar coordinates is 
% transformed to ENU coordinates


%% Sick Lidar - Case 1

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.


% A point in sick lidar coordinates.  
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'sickLidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sick Lidar - Case 2

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.


% A point in sick lidar coordinates.  
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'sickLidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sick Lidar - Case 3

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.


% A point in sick lidar coordinates.  
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'sick_lidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sick Lidar - Case 4

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.


% A point in sick lidar coordinates.  
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'sick_lidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Sick Lidar - Case 5

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.


% A point in sick lidar coordinates.  
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'sick_lidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sick Lidar - Case 6

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.


% A point in sick lidar coordinates.  
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'sick_lidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));


%% Sick Lidar - Case 7

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.


% A point in sick lidar coordinates.  
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'sick_lidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));


%% Sick Lidar - Case 8

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.


% A point in sick lidar coordinates.  
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'sick_lidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));


%% Sick Lidar - Case 9

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.


% A point in sick lidar coordinates.  
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'sick_lidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sick Lidar - Case 10

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.

% A point in sick lidar coordinates. 
sensorReading_SensorCoord = [6.5, 4, 4.4];

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
from_dashCoord = 'sickLidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [3, 4, -5];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Sick Lidar - Case 11

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.


% A point in sick lidar coordinates. 
sensorReading_SensorCoord = [-11.5, -6, 2.4];

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
from_dashCoord = 'sickLidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [1, -6, 8];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sick Lidar - Case 12

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.


% A point in sick lidar coordinates. 
sensorReading_SensorCoord = [0.5, 6, -2.6];

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
from_dashCoord = 'sick_lidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [-4, 2, 1];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sick Lidar - Case 13

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.


% A point in sick lidar coordinates. 
sensorReading_SensorCoord = [4.5, 6, 1.4];

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
from_dashCoord = 'sick_lidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [2, 6, -3];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Sick Lidar - Case 14

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.


% A point in sick lidar coordinates.
sensorReading_SensorCoord = [6.5, 3, 8.4];

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
from_dashCoord = 'sick_lidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [1, 4, 3];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sick Lidar - Case 15

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.


% A point in sick lidar coordinates.
sensorReading_SensorCoord = [11, 10.5, -5.6];

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
from_dashCoord = 'sick_lidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [-6, 2, 7];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Sick Lidar - Case 16

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.

% A point in sick lidar coordinates. 
sensorReading_SensorCoord = [-8.4, -1, -0.5];

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
from_dashCoord = 'sick_lidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [3, 6, 1];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sick Lidar - Case 17

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.


% A point in sick lidar coordinates. 
sensorReading_SensorCoord = [2.5, -1.4, 16];

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
from_dashCoord = 'sick_lidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [3, 9, 8];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sick Lidar - Case 18

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.


% A point in sick lidar coordinates. 
sensorReading_SensorCoord = [12.4, 1.5, -3];

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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'sick_lidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [8, 1, -3];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% The following cases were written to test the accuracy of velodyne lidar coordinates 

% when a point(sensorReading_SensorCoord) in velodyne lidar coordinates is 
% transformed to ENU coordinates


%% Velodyne Lidar - Case 1

% In this case, we are transforming the sensorReading_SensorCoord 
% (velodyne lidar coordinates) into ENU coordinates.


% A point in velodyne coordinates.  
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'velodyne';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Velodyne Lidar - Case 2

% In this case, we are transforming the sensorReading_SensorCoord 
% (velodyne lidar coordinates) into ENU coordinates.


% A point in velodyne coordinates.  
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'velodyne_lidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Velodyne Lidar - Case 3

% In this case, we are transforming the sensorReading_SensorCoord 
% (velodyne lidar coordinates) into ENU coordinates.


% A point in velodyne coordinates.  
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'velodyneLidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Velodyne Lidar - Case 4

% In this case, we are transforming the sensorReading_SensorCoord 
% (velodyne lidar coordinates) into ENU coordinates.


% A point in velodyne coordinates.  
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'velodynelidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Velodyne Lidar - Case 5

% In this case, we are transforming the sensorReading_SensorCoord 
% (velodyne lidar coordinates) into ENU coordinates.


% A point in velodyne coordinates.  
sensorReading_SensorCoord = [8, -1.4425, 3.2064];

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
from_dashCoord = 'velodynelidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Velodyne Lidar - Case 6

% In this case, we are transforming the sensorReading_SensorCoord 
% (velodyne lidar coordinates) into ENU coordinates.


% A point in velodyne coordinates.  
sensorReading_SensorCoord = [8.5575, 5, -3.7936];

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
from_dashCoord = 'velodynelidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Velodyne Lidar - Case 7

% In this case, we are transforming the sensorReading_SensorCoord 
% (velodyne lidar coordinates) into ENU coordinates.


% A point in velodyne coordinates.  
sensorReading_SensorCoord = [-1.2064, -7.4425, -1];

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
from_dashCoord = 'velodynelidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Velodyne Lidar - Case 8

% In this case, we are transforming the sensorReading_SensorCoord 
% (velodyne lidar coordinates) into ENU coordinates.


% A point in velodyne coordinates.  
sensorReading_SensorCoord = [9, 5.7936, 6.5575];

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
from_dashCoord = 'velodynelidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Velodyne Lidar - Case 9

% In this case, we are transforming the sensorReading_SensorCoord 
% (velodyne lidar coordinates) into ENU coordinates.


% A point in velodyne coordinates.  
sensorReading_SensorCoord = [0.2064, -3, -4.4425];

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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'velodynelidar';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% The following cases were written to test the accuracy of left GPS coordinates 

% when a point(sensorReading_SensorCoord) in left GPS coordinates is 
% transformed to ENU coordinates


%% Left GPS - Case 1

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.

% A point in left GPS coordinates.  
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'leftGPS';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Left GPS - Case 2

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.


% A point in left GPS coordinates.  
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'left GPS';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Left GPS - Case 3

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.


% A point in left GPS coordinates.  
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'left_GPS';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Left GPS - Case 4

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.


% A point in left GPS coordinates.  
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'leftgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Left GPS - Case 5

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.


% A point in left GPS coordinates.  
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'leftgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Left GPS - Case 6

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.


% A point in left GPS coordinates.  
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'leftgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Left GPS - Case 7

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.


% A point in left GPS coordinates.  
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'leftgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Left GPS - Case 8

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.


% A point in left GPS coordinates.  
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'leftgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Left GPS - Case 9

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.


% A point in left GPS coordinates.  
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'leftgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Left GPS - Case 10

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.

% A point in left GPS coordinates. 
sensorReading_SensorCoord = [6.7, -4.762, 6.9];

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
from_dashCoord = 'leftGPS';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [6, -4, 9];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Left GPS - Case 11

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.


% A point in left GPS coordinates. 
sensorReading_SensorCoord = [7.7, 3.238, -3.1];

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
from_dashCoord = 'left GPS';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [12, 4, -1];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Left GPS - Case 12

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.


% A point in left GPS coordinates. 
sensorReading_SensorCoord = [-7.3, 12.238, 0.9];

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
from_dashCoord = 'left_GPS';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [-8, 9, 3];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Left GPS - Case 13

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.


% A point in left GPS coordinates. 
sensorReading_SensorCoord = [6.7, 1.238, 4.9];

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
from_dashCoord = 'leftgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [6, 2, 9];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Left GPS - Case 14

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.


% A point in left GPS coordinates. 
sensorReading_SensorCoord = [19.7, -6.762, 7.9];

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
from_dashCoord = 'leftgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [11, -5, 4];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Left GPS - Case 15

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.


% A point in left GPS coordinates. 
sensorReading_SensorCoord = [7.238, -0.7, 5.9];

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
from_dashCoord = 'leftgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [5, -1, 9];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Left GPS - Case 16

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.


% A point in left GPS coordinates. 
sensorReading_SensorCoord = [-9.9, -5.762, -9.3];

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
from_dashCoord = '4leftgps4';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [-9, 2, 8];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Left GPS - Case 17

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.


% A point in left GPS coordinates. 
sensorReading_SensorCoord = [15.7, 4.1, 3.238];

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
from_dashCoord = 'left123gps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [6, -3, 1];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Left GPS - Case 18

% In this case, we are transforming the sensorReading_SensorCoord 
% (left GPS coordinates) into ENU coordinates.


% A point in left GPS coordinates. 
sensorReading_SensorCoord = [4.9, -4.3, 4.238];

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
from_dashCoord = 'left - gps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [-2, 9, 4];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% The following cases were written to test the accuracy of right GPS coordinates 

% when a point(sensorReading_SensorCoord) in right GPS coordinates is 
% transformed to ENU coordinates


%% right GPS - Case 1

% In this case, we are transforming the sensorReading_SensorCoord 
% (right GPS coordinates) into ENU coordinates.


% A point in right GPS coordinates.  
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'rightGPS';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% right GPS - Case 2

% In this case, we are transforming the sensorReading_SensorCoord 
% (right GPS coordinates) into ENU coordinates.


% A point in right GPS coordinates.  
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'right GPS';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% right GPS - Case 3

% In this case, we are transforming the sensorReading_SensorCoord 
% (right GPS coordinates) into ENU coordinates.


% A point in right GPS coordinates.  
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'right_GPS';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% right GPS - Case 4

% In this case, we are transforming the sensorReading_SensorCoord 
% (right GPS coordinates) into ENU coordinates.


% A point in right GPS coordinates.  
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'rightgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% right GPS - Case 5

% In this case, we are transforming the sensorReading_SensorCoord 
% (right GPS coordinates) into ENU coordinates.


% A point in right GPS coordinates.  
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'rightgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% right GPS - Case 6

% In this case, we are transforming the sensorReading_SensorCoord 
% (right GPS coordinates) into ENU coordinates.


% A point in right GPS coordinates.  
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'rightgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% right GPS - Case 7

% In this case, we are transforming the sensorReading_SensorCoord 
% (right GPS coordinates) into ENU coordinates.


% A point in right GPS coordinates.  
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'rightgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% right GPS - Case 8

% In this case, we are transforming the sensorReading_SensorCoord 
% (right GPS coordinates) into ENU coordinates.


% A point in right GPS coordinates.  
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'rightgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% right GPS - Case 9

% In this case, we are transforming the sensorReading_SensorCoord 
% (right GPS coordinates) into ENU coordinates.


% A point in right GPS coordinates.  
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

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'rightgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% right GPS - Case 10

% In this case, we are transforming the sensorReading_SensorCoord 
% (right GPS coordinates) into ENU coordinates.


% A point in right GPS coordinates. 
sensorReading_SensorCoord = [6.7, -3.238, 6.9];

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
% "fcn_Transform_ENUToSensorCoord" cases

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'rightGPS';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [6, -4, 9];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% right GPS - Case 11

% In this case, we are transforming the sensorReading_SensorCoord 
% (right GPS coordinates) into ENU coordinates.


% A point in right GPS coordinates. 
sensorReading_SensorCoord = [7.7, 4.762, -3.1];

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
% "fcn_Transform_ENUToSensorCoord" cases

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'right GPS';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [12, 4, -1];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% right GPS - Case 12

% In this case, we are transforming the sensorReading_SensorCoord 
% (right GPS coordinates) into ENU coordinates.


% A point in right GPS coordinates. 
sensorReading_SensorCoord = [-7.3, 13.762, 0.9];

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
% "fcn_Transform_ENUToSensorCoord" cases

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'right_GPS';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [-8, 9, 3];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% right GPS - Case 13

% In this case, we are transforming the sensorReading_SensorCoord 
% (right GPS coordinates) into ENU coordinates.


% A point in right GPS coordinates. 
sensorReading_SensorCoord = [6.7, 2.762, 4.9];

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
% "fcn_Transform_ENUToSensorCoord" cases

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'rightgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [6, 2, 9];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% right GPS - Case 14

% In this case, we are transforming the sensorReading_SensorCoord 
% (right GPS coordinates) into ENU coordinates.


% A point in right GPS coordinates. 
sensorReading_SensorCoord = [19.7, -5.238, 7.9];

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
% "fcn_Transform_ENUToSensorCoord" cases

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'rightgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [11, -5, 4];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% right GPS - Case 15

% In this case, we are transforming the sensorReading_SensorCoord 
% (right GPS coordinates) into ENU coordinates.


% A point in right GPS coordinates. 
sensorReading_SensorCoord = [8.762, -0.7, 5.9];

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
% "fcn_Transform_ENUToSensorCoord" cases

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'rightgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [5, -1, 9];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% right GPS - Case 16

% In this case, we are transforming the sensorReading_SensorCoord 
% (right GPS coordinates) into ENU coordinates.

% A point in right GPS coordinates. 
sensorReading_SensorCoord = [-9.9, -4.238, -9.3];

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
% "fcn_Transform_ENUToSensorCoord" cases

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'rightgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [-9, 2, 8];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% right GPS - Case 17

% In this case, we are transforming the sensorReading_SensorCoord 
% (right GPS coordinates) into ENU coordinates.

% A point in right GPS coordinates. 
sensorReading_SensorCoord = [15.7, 4.1, 4.762];

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
% "fcn_Transform_ENUToSensorCoord" cases

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'rightgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [6, -3, 1];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% right GPS - Case 18

% In this case, we are transforming the sensorReading_SensorCoord 
% (right GPS coordinates) into ENU coordinates.

% A point in right GPS coordinates. 
sensorReading_SensorCoord = [4.9, -4.3, 5.762];

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
% "fcn_Transform_ENUToSensorCoord" cases

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'rightgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [-2, 9, 4];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Fail Conditions 

% input cannot be NaN

if 1 == 0

 % A point in sensor coordinates.  
sensorReading_SensorCoord = [NaN, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [3,4,-3,NaN,270,90];

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'rightgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0.9, -2.3, -3.238];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

end

%% 

% input cannot be inf

if 1 == 0

 % A point in sensor coordinates.  
sensorReading_SensorCoord = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,4,-3,0,270,inf];

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'rightgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0.9, -2.3, -3.238];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

end

%% 

% input cannot be inf or NaN

if 1 == 0

 % A point in sensor coordinates.  
sensorReading_SensorCoord = [0, NaN, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [inf,4,-3,0,NaN,0];

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'rightgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0.9, -2.3, -3.238];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

end

%% 

% sensorReading_ENU must be 1 x 3

if 1 == 0

 % A point in sensor coordinates.  
sensorReading_SensorCoord = [0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [3,4,-3,180,270,90];


% Transforms this coordinates into ENU coordinates
from_dashCoord = 'rightgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0.9, -2.3, -3.238];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

end

%% 

% sensorReading_ENU must be 1 x 6

if 1 == 0

 % A point in sensor coordinates.  
sensorReading_SensorCoord = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [3,4,-3,180,270];


% Transforms this coordinates into ENU coordinates
from_dashCoord = 'rightgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0.9, -2.3, -3.238];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

end

%% 

% input arguments should not exceed 3

if 1 == 0

 % A point in sensor coordinates.  
sensorReading_SensorCoord = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [3,4,-3,180,270];

SensorVals = [3, 4, 6];

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'rightgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, vehiclePose_ENU, from_dashCoord, SensorVals);
expected_transformed_ENUPoint_from_dashCoord = [0.9, -2.3, -3.238];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

end

%% 

% input arguments should not be less than 3

if 1 == 0

 % A point in sensor coordinates.  
sensorReading_SensorCoord = [0, 0, 0];

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'rightgps';  

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(sensorReading_SensorCoord, from_dashCoord);
expected_transformed_ENUPoint_from_dashCoord = [0.9, -2.3, -3.238];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

end