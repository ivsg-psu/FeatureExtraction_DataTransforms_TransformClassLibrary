% script_test_fcn_findVehiclePoseinENU.m
% tests fcn_findVehiclePoseinENU.m

% Revision history
% 2023_06_29 - Aneesh Batchu
% -- wrote the code originally

%% Set up the workspace

clc
close all

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

%% Assumptions

% The vehicle is assumed to have zero pitch in ENU coordinates.
%
% Therefore, the output of this code will be the position of the
% vehicle,[X_vehicle_ENU, Y_vehicle_ENU, Z_vehicle_ENU], and 
%  ROLL_vehicle_ENU, 0, YAW_vehicle_ENU]
%
% vehiclePose_ENU = [X_vehicle_ENU, Y_vehicle_ENU, Z_vehicle_ENU, 
%                   ROLL_vehicle_ENU (in degrees), 0, YAW_vehicle_ENU(in degrees)]
%
% SensorMount_offset_y_relative_to_VehicleOrigin = 0. 
%
%
% The distance between the GPS Antennas mid-point and the Sensor Mount 
% is assumed to be zero

%% Test 


% The centers of Left GPS Antenna
% GPSLeft_ENU = [x, y, z] in meters
GPSLeft_ENU = [1, 1, 2; -1, -1, 2; 1, 3, 2; 1, 1, 2; 3, 3, 2];

% The centers of Right GPS Antenna
% GPSRight_ENU = [x, y, z] in meters
GPSRight_ENU = [1, 3, 2; 3, 3, 2; 1, 1, 2; 1, 3, 1; 1, 1, 2];
    
% The pitch of the vehicle is assumed as zero
PITCH_vehicle_ENU = zeros(size(GPSLeft_ENU,1),1);

% The sensor mount offset relative to vehicle origin = 
% [-X_SensorMount_center, 0, +Z_SensorMount_center] in meters
SensorMount_offset_relative_to_VehicleOrigin = [-1 0 1.6]; 


% The POSE of the vehicle in ENU coordinates is in the form [X_vehicle_ENU, 
% Y_vehicle_ENU, Z_vehicle_ENU, 0, 0, YAW_vehicle_ENU]

vehiclePose_ENU = fcn_Transform_findVehiclePoseinENU(GPSLeft_ENU, GPSRight_ENU, PITCH_vehicle_ENU, SensorMount_offset_relative_to_VehicleOrigin);

disp(vehiclePose_ENU)

%% Case 1 

% This case has the inputs as per the assumptions 

figure(1);
clf;
grid on
hold on
axis equal
xlabel('East', 'FontSize',15, 'FontWeight','bold')
ylabel('North', 'FontSize',15, 'FontWeight','bold')
zlabel('Up', 'FontSize',15, 'FontWeight','bold')
view(3)
axis equal

% The center of Left GPS Antenna
% GPSLeft_ENU = [x, y, z] in meters
GPSLeft_ENU =[1, 1, 2];

% The center of Right GPS Antenna
% GPSRight_ENU = [x, y, z] in meters
GPSRight_ENU =[3, 3, 2];

% The sensor mount offset relative to vehicle origin = 
% [-X_SensorMount_center, 0, +Z_SensorMount_center] in meters
SensorMount_offset_relative_to_VehicleOrigin = [-1 0 1.6]; 

PITCH_vehicle_ENU = 0;

% The POSE of the vehicle in ENU coordinates is in the form [X_vehicle_ENU, 
% Y_vehicle_ENU, Z_vehicle_ENU, 0, 0, YAW_vehicle_ENU]
vehiclePose_ENU = fcn_Transform_findVehiclePoseinENU(GPSLeft_ENU, GPSRight_ENU, PITCH_vehicle_ENU, SensorMount_offset_relative_to_VehicleOrigin);

% This is the hand calculated result 
HandCalculatedValue_ENU = [2-(1/sqrt(2)), 2+(1/sqrt(2)), 2-1.6, 0, 0, 135];

% assertion to test whether the Hand calculated Vehicle Pose matches with
% function's vehicle Pose
assert(isequal(vehiclePose_ENU, HandCalculatedValue_ENU));

%% Case 2

figure(2);
clf;
grid on
hold on
axis equal
xlabel('East', 'FontSize',15, 'FontWeight','bold')
ylabel('North', 'FontSize',15, 'FontWeight','bold')
zlabel('Up', 'FontSize',15, 'FontWeight','bold')
view(3)
axis equal

% This case has a different SensorMount_offset_x_relative_to_VehicleOrigin
% compared to case 1.

% The center of Left GPS Antenna
% GPSLeft_ENU = [x, y, z] in meters
GPSLeft_ENU =[1, 1, 2];

% The center of Right GPS Antenna
% GPSRight_ENU = [x, y, z] in meters
GPSRight_ENU =[3, 3, 2];

% The sensor mount offset relative to vehicle origin = 
% [-X_SensorMount_center, 0, +Z_SensorMount_center]
SensorMount_offset_relative_to_VehicleOrigin = [-2 0 1.6];

PITCH_vehicle_ENU = 0;

% The POSE of the vehicle in ENU coordinates is in the form [X_vehicle_ENU, 
% Y_vehicle_ENU, Z_vehicle_ENU, 0, 0, YAW_vehicle_ENU]
vehiclePose_ENU = fcn_Transform_findVehiclePoseinENU(GPSLeft_ENU, GPSRight_ENU, PITCH_vehicle_ENU, SensorMount_offset_relative_to_VehicleOrigin);

% This is the hand calculated result 
HandCalculatedValue_ENU = [2-(2/sqrt(2)), 2+(2/sqrt(2)), 2-1.6, 0, 0, 135];

% assertion to test whether the Hand calculated Vehicle Pose matches with
% function's vehicle Pose
assert(isequal(vehiclePose_ENU, HandCalculatedValue_ENU));

%% Case 3

% This case has a different SensorMount_offset_z_relative_to_VehicleOrigin
% compared to case 1.

figure(3);
clf;
grid on
hold on
axis equal
xlabel('East', 'FontSize',15, 'FontWeight','bold')
ylabel('North', 'FontSize',15, 'FontWeight','bold')
zlabel('Up', 'FontSize',15, 'FontWeight','bold')
view(3)
axis equal

% The center of Left GPS Antenna
% GPSLeft_ENU = [x, y, z] in meters
GPSLeft_ENU =[1, 1, 2];

% The center of Right GPS Antenna
% GPSRight_ENU = [x, y, z] in meters
GPSRight_ENU =[3, 3, 2];

% The sensor mount offset relative to vehicle origin = 
% [-X_SensorMount_center, 0, +Z_SensorMount_center]
SensorMount_offset_relative_to_VehicleOrigin = [-1 0 1.9]; 

PITCH_vehicle_ENU = 0;

% The POSE of the vehicle in ENU coordinates is in the form [X_vehicle_ENU, 
% Y_vehicle_ENU, Z_vehicle_ENU, 0, 0, YAW_vehicle_ENU]
vehiclePose_ENU = fcn_Transform_findVehiclePoseinENU(GPSLeft_ENU, GPSRight_ENU, PITCH_vehicle_ENU, SensorMount_offset_relative_to_VehicleOrigin);

% This is the hand calculated result 
HandCalculatedValue_ENU = [2-(1/sqrt(2)), 2+(1/sqrt(2)), 2-1.9, 0, 0, 135];

% assertion to test whether the Hand calculated Vehicle Pose matches with
% function's vehicle Pose
assert(isequal(vehiclePose_ENU, HandCalculatedValue_ENU));

%% Case 4

figure(4);
clf;
grid on
hold on
axis equal
xlabel('East', 'FontSize',15, 'FontWeight','bold')
ylabel('North', 'FontSize',15, 'FontWeight','bold')
zlabel('Up', 'FontSize',15, 'FontWeight','bold')
view(3)
axis equal

% This case has a different SensorMount_offset_x_relative_to_VehicleOrigin 
% and SensorMount_offset_z_relative_to_VehicleOrigin compared to case 1.

% The center of Left GPS Antenna
% GPSLeft_ENU = [x, y, z] in meters
GPSLeft_ENU =[1, 1, 2];

% The center of Right GPS Antenna
% GPSRight_ENU = [x, y, z] in meters
GPSRight_ENU =[3, 3, 2];

% The sensor mount offset relative to vehicle origin = 
% [-X_SensorMount_center, 0, +Z_SensorMount_center]
SensorMount_offset_relative_to_VehicleOrigin = [-2.3 0 1.45]; 

PITCH_vehicle_ENU = 0;

% The POSE of the vehicle in ENU coordinates is in the form [X_vehicle_ENU, 
% Y_vehicle_ENU, Z_vehicle_ENU, 0, 0, YAW_vehicle_ENU]
vehiclePose_ENU = fcn_Transform_findVehiclePoseinENU(GPSLeft_ENU, GPSRight_ENU, PITCH_vehicle_ENU, SensorMount_offset_relative_to_VehicleOrigin);

% This is the hand calculated result 
HandCalculatedValue_ENU = [2-(2.3/sqrt(2)), 2+(2.3/sqrt(2)), 2-1.45, 0, 0, 135];

% assertion to test whether the Hand calculated Vehicle Pose matches with
% function's vehicle Pose
assert(isequal(vehiclePose_ENU, HandCalculatedValue_ENU));

%% Case 5

% This case has a different GPSRight_ENU compared to case 1

figure(5);
clf;
grid on
hold on
axis equal
xlabel('East', 'FontSize',15, 'FontWeight','bold')
ylabel('North', 'FontSize',15, 'FontWeight','bold')
zlabel('Up', 'FontSize',15, 'FontWeight','bold')
view(3)
axis equal

% The center of Left GPS Antenna
% GPSLeft_ENU = [x, y, z] in meters
GPSLeft_ENU =[1, 1, 2];

% The center of Right GPS Antenna
% GPSRight_ENU = [x, y, z] in meters
GPSRight_ENU =[5, 5, 2];

% The sensor mount offset relative to vehicle origin = 
% [-X_SensorMount_center, 0, +Z_SensorMount_center]
SensorMount_offset_relative_to_VehicleOrigin = [-1 0 1.6]; 

PITCH_vehicle_ENU = 0;

% The POSE of the vehicle in ENU coordinates is in the form [X_vehicle_ENU, 
% Y_vehicle_ENU, Z_vehicle_ENU, 0, 0, YAW_vehicle_ENU]
vehiclePose_ENU = fcn_Transform_findVehiclePoseinENU(GPSLeft_ENU, GPSRight_ENU, PITCH_vehicle_ENU, SensorMount_offset_relative_to_VehicleOrigin);

% This is the hand calculated result 
HandCalculatedValue_ENU = [3-(1/sqrt(2)), 3+(1/sqrt(2)), 2-1.6, 0, 0, 135];

% assertion to test whether the Hand calculated Vehicle Pose matches with
% function's vehicle Pose
assert(isequal(vehiclePose_ENU, HandCalculatedValue_ENU));

%% Case 6

% This case has a different GPSLeft_ENU compared to case 1

figure(6);
clf;
grid on
hold on
axis equal
xlabel('East', 'FontSize',15, 'FontWeight','bold')
ylabel('North', 'FontSize',15, 'FontWeight','bold')
zlabel('Up', 'FontSize',15, 'FontWeight','bold')
view(3)
axis equal

% The center of Left GPS Antenna
% GPSLeft_ENU = [x, y, z] in meters
GPSLeft_ENU =[-1, -1, 2];

% The center of Right GPS Antenna
% GPSRight_ENU = [x, y, z] in meters
GPSRight_ENU =[3, 3, 2];

% The sensor mount offset relative to vehicle origin = 
% [-X_SensorMount_center, 0, +Z_SensorMount_center]
SensorMount_offset_relative_to_VehicleOrigin = [-1 0 1.6]; 

PITCH_vehicle_ENU = 0;

% The POSE of the vehicle in ENU coordinates is in the form [X_vehicle_ENU, 
% Y_vehicle_ENU, Z_vehicle_ENU, 0, 0, YAW_vehicle_ENU]
vehiclePose_ENU = fcn_Transform_findVehiclePoseinENU(GPSLeft_ENU, GPSRight_ENU, PITCH_vehicle_ENU, SensorMount_offset_relative_to_VehicleOrigin);

% This is the hand calculated result 
HandCalculatedValue_ENU = [1-(1/sqrt(2)), 1+(1/sqrt(2)), 2-1.6, 0, 0, 135];

% assertion to test whether the Hand calculated Vehicle Pose matches with
% function's vehicle Pose
assert(isequal(vehiclePose_ENU, HandCalculatedValue_ENU));

%% Case 7

% This case has a different GPSRight_ENU compared to case 1 and the
% vehicle's YAW in ENU coordinates is 180 degrees

figure(7);
clf;
grid on
hold on
axis equal
xlabel('East', 'FontSize',15, 'FontWeight','bold')
ylabel('North', 'FontSize',15, 'FontWeight','bold')
zlabel('Up', 'FontSize',15, 'FontWeight','bold')
view(3)
axis equal

% The center of Left GPS Antenna
% GPSLeft_ENU = [x, y, z] in meters
GPSLeft_ENU =[1, 1, 2];

% The center of Right GPS Antenna
% GPSRight_ENU = [x, y, z] in meters
GPSRight_ENU =[1, 3, 2];

% The sensor mount offset relative to vehicle origin = 
% [-X_SensorMount_center, 0, +Z_SensorMount_center]
SensorMount_offset_relative_to_VehicleOrigin = [-1 0 1.6]; 

PITCH_vehicle_ENU = 0;

% The POSE of the vehicle in ENU coordinates is in the form [X_vehicle_ENU, 
% Y_vehicle_ENU, Z_vehicle_ENU, 0, 0, YAW_vehicle_ENU]
vehiclePose_ENU = fcn_Transform_findVehiclePoseinENU(GPSLeft_ENU, GPSRight_ENU, PITCH_vehicle_ENU, SensorMount_offset_relative_to_VehicleOrigin);

% This is the hand calculated result 
HandCalculatedValue_ENU = [0, 2, 2-1.6, 0, 0, 180];

% assertion to test whether the Hand calculated Vehicle Pose matches with
% function's vehicle Pose
assert(isequal(vehiclePose_ENU, HandCalculatedValue_ENU));

%% Case 8

% This case has a different GPSLeft_ENU compared to case 1 and the
% vehicle's YAW in ENU coordinates is zero

figure(8);
clf;
grid on
hold on
axis equal
xlabel('East', 'FontSize',15, 'FontWeight','bold')
ylabel('North', 'FontSize',15, 'FontWeight','bold')
zlabel('Up', 'FontSize',15, 'FontWeight','bold')
view(3)
axis equal

% The center of Left GPS Antenna
% GPSLeft_ENU = [x, y, z] in meters
GPSLeft_ENU =[1, 3, 2];

% The center of Right GPS Antenna
% GPSRight_ENU = [x, y, z] in meters
GPSRight_ENU =[1, 1, 2];

% The sensor mount offset relative to vehicle origin = 
% [-X_SensorMount_center, 0, +Z_SensorMount_center]
SensorMount_offset_relative_to_VehicleOrigin = [-1 0 1.6]; 

PITCH_vehicle_ENU = 0;

% The POSE of the vehicle in ENU coordinates is in the form [X_vehicle_ENU, 
% Y_vehicle_ENU, Z_vehicle_ENU, 0, 0, YAW_vehicle_ENU]
vehiclePose_ENU = fcn_Transform_findVehiclePoseinENU(GPSLeft_ENU, GPSRight_ENU, PITCH_vehicle_ENU, SensorMount_offset_relative_to_VehicleOrigin);

% This is the hand calculated result 
HandCalculatedValue_ENU = [2, 2, 2-1.6, 0, 0, 0];

% assertion to test whether the Hand calculated Vehicle Pose matches with
% function's vehicle Pose
assert(isequal(vehiclePose_ENU, HandCalculatedValue_ENU));

%% Case 9

% This case has the inputs as per the assumptions 

figure(9);
clf;
grid on
hold on
axis equal
xlabel('East', 'FontSize',15, 'FontWeight','bold')
ylabel('North', 'FontSize',15, 'FontWeight','bold')
zlabel('Up', 'FontSize',15, 'FontWeight','bold')
view(3)
axis equal

% The center of Left GPS Antenna
% GPSLeft_ENU = [x, y, z] in meters
GPSLeft_ENU =[1, 1, 2];

% The center of Right GPS Antenna
% GPSRight_ENU = [x, y, z] in meters
GPSRight_ENU =[1, 3, 1];

% The sensor mount offset relative to vehicle origin = 
% [-X_SensorMount_center, 0, +Z_SensorMount_center] in meters
SensorMount_offset_relative_to_VehicleOrigin = [-1 0 1.6]; 

PITCH_vehicle_ENU = 0;

% The POSE of the vehicle in ENU coordinates is in the form [X_vehicle_ENU, 
% Y_vehicle_ENU, Z_vehicle_ENU, 0, 0, YAW_vehicle_ENU]
vehiclePose_ENU = fcn_Transform_findVehiclePoseinENU(GPSLeft_ENU, GPSRight_ENU, PITCH_vehicle_ENU, SensorMount_offset_relative_to_VehicleOrigin);

% This is the hand calculated result 
BasedOnFindVehiclePose4_ENU = [0.1056, 2, -0.5472, -26.5651, 0, 180.000];

% assertion to test whether the Hand calculated Vehicle Pose matches with
% function's vehicle Pose
% assert(isequal(vehiclePose_ENU, BasedOnFindVehiclePose4_ENU));

%% Case 10

% This case has a different GPSLeft_ENU compared to case 1 and the
% vehicle's YAW in ENU coordinates is zero


% The center of Left GPS Antenna
% GPSLeft_ENU = [x, y, z] in meters
GPSLeft_ENU =[1, 3, 2];

% The center of Right GPS Antenna
% GPSRight_ENU = [x, y, z] in meters
GPSRight_ENU =[1, 1, 2];

antenna_baseline = sum((GPSRight_ENU-GPSLeft_ENU).^2,2).^0.5;

figure(10);
clf;
grid on
hold on
axis equal
xlabel('East', 'FontSize',15, 'FontWeight','bold')
ylabel('North', 'FontSize',15, 'FontWeight','bold')
zlabel('Up', 'FontSize',15, 'FontWeight','bold')
view(3)
axis([-8 8 -8 8 0 2.5]);

for ith_angle = 0:5:360
    angle_in_rads = ith_angle*pi/180;
    GPSRight_ENU = GPSLeft_ENU + [cos(angle_in_rads) sin(angle_in_rads) 0]*antenna_baseline;

    % The sensor mount offset relative to vehicle origin =
    % [-X_SensorMount_center, 0, +Z_SensorMount_center]
    SensorMount_offset_relative_to_VehicleOrigin = [-1 0 1.6];

    PITCH_vehicle_ENU = 0;

    % The POSE of the vehicle in ENU coordinates is in the form [X_vehicle_ENU,
    % Y_vehicle_ENU, Z_vehicle_ENU, 0, 0, YAW_vehicle_ENU]
    vehiclePose_ENU = fcn_Transform_findVehiclePoseinENU(GPSLeft_ENU, GPSRight_ENU, PITCH_vehicle_ENU, SensorMount_offset_relative_to_VehicleOrigin);

    pause(0.015);
end

%% Case 

% This case has a different GPSLeft_ENU compared to case 1 and the
% vehicle's YAW in ENU coordinates is zero

% The center of Left GPS Antenna
% GPSLeft_ENU = [x, y, z] in meters
GPSLeft_ENU =[1, 3, 2];

% The center of Right GPS Antenna
% GPSRight_ENU = [x, y, z] in meters
GPSRight_ENU =[1, 1, 2];

antenna_baseline = sum((GPSRight_ENU-GPSLeft_ENU).^2,2).^0.5;

figure(11);
clf;
grid on
hold on
axis equal
xlabel('East', 'FontSize',15, 'FontWeight','bold')
ylabel('North', 'FontSize',15, 'FontWeight','bold')
zlabel('Up', 'FontSize',15, 'FontWeight','bold')
view(3)
axis([-8 8 -8 8 -8 8]);

for ith_angle = 0:5:360
    angle_in_rads = ith_angle*pi/180;
    GPSRight_ENU = GPSLeft_ENU + [0 cos(angle_in_rads) sin(angle_in_rads)]*antenna_baseline;

    % The sensor mount offset relative to vehicle origin =
    % [-X_SensorMount_center, 0, +Z_SensorMount_center]
    SensorMount_offset_relative_to_VehicleOrigin = [-1 0 1.6];

    PITCH_vehicle_ENU = 0;

    % The POSE of the vehicle in ENU coordinates is in the form [X_vehicle_ENU,
    % Y_vehicle_ENU, Z_vehicle_ENU, 0, 0, YAW_vehicle_ENU]
    vehiclePose_ENU = fcn_Transform_findVehiclePoseinENU(GPSLeft_ENU, GPSRight_ENU, PITCH_vehicle_ENU, SensorMount_offset_relative_to_VehicleOrigin);

end


%% Fail Conditions

if 1==0
    % The center of Left GPS Antenna
    % GPSLeft_ENU = [x, y] in meters
    GPSLeft_ENU =[1, 3];

    % The center of Right GPS Antenna
    % GPSRight_ENU = [x, y] in meters
    GPSRight_ENU =[1, 1];

    % The sensor mount offset relative to vehicle origin =
    % [-X_SensorMount_center, 0, +Z_SensorMount_center]
    SensorMount_offset_relative_to_VehicleOrigin = [-1 0 1.6];

    % The POSE of the vehicle in ENU coordinates is in the form [X_vehicle_ENU,
    % Y_vehicle_ENU, Z_vehicle_ENU, 0, 0, YAW_vehicle_ENU]
    vehiclePose_ENU = fcn_findVehiclePoseinENU(GPSLeft_ENU, GPSRight_ENU, SensorMount_offset_relative_to_VehicleOrigin);
end

%% Fail Conditions

if 1==0
    % The center of Left GPS Antenna
    % GPSLeft_ENU = [x, y] in meters
    GPSLeft_ENU =[1, 3, 0];

    % The center of Right GPS Antenna
    % GPSRight_ENU = [x, y] in meters
    GPSRight_ENU =[1, 1, 0];

    % The sensor mount offset relative to vehicle origin =
    % [-X_SensorMount_center, 0, +Z_SensorMount_center]
    SensorMount_offset_relative_to_VehicleOrigin = [-1 0];

    % The POSE of the vehicle in ENU coordinates is in the form [X_vehicle_ENU,
    % Y_vehicle_ENU, Z_vehicle_ENU, 0, 0, YAW_vehicle_ENU]
    vehiclePose_ENU = fcn_findVehiclePoseinENU(GPSLeft_ENU, GPSRight_ENU, SensorMount_offset_relative_to_VehicleOrigin);
end

%% Fail Conditions

if 1==0
    % The center of Left GPS Antenna
    % GPSLeft_ENU = [x, y] in meters
    GPSLeft_ENU =[1, 3, 0];

    % The center of Right GPS Antenna
    % GPSRight_ENU = [x, y] in meters
    GPSRight_ENU =[1, 1, 0];

    % The sensor mount offset relative to vehicle origin =
    % [-X_SensorMount_center, 0, +Z_SensorMount_center]
    SensorMount_offset_relative_to_VehicleOrigin = [-1 0 1.6];

    % The POSE of the vehicle in ENU coordinates is in the form [X_vehicle_ENU,
    % Y_vehicle_ENU, Z_vehicle_ENU, 0, 0, YAW_vehicle_ENU]
    vehiclePose_ENU = fcn_findVehiclePoseinENU(GPSLeft_ENU, GPSRight_ENU);
end

