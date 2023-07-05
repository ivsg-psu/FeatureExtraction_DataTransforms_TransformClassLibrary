function transformed_ENUPoint_in_SensorCoord = fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, sensor)
% fcn_Transform_ENUToSensorCoord
%
% This function takes an ENU point (sensor reading in ENU coordinates),
% vehicle pose in ENU coordinates, and the sensor string as the inputs and
% outputs the ENU point's reading (sensorReading_ENU) in sensor coordinates
%
% 
% FORMAT:
%
%      transformed_ENUPoint_in_SensorCoord = fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, sensor)
%
% INPUTS:
%      
%      sensorReading_ENU: Sensor reading in ENU coordinates.
%       
%      vehiclePose_ENU: [x,y,z,roll,pitch,yaw]
% 
%        position of the vehicle:
% 
%           x: translates the vehicle in the x direction relative to
%           ENU coordinates
%           y: translates the vehicle in the y direction relative to
%           ENU coordinates
%           z: translates the vehicle in the z direction relative to
%           ENU coordinates
% 
%        orientation of the vehicle - Follows ISO convention
% 
%          roll: rotates the vehicle about its x-axis relative to ENU
%          coordinates (the vehicle's orientation changes relative to
%          the Earth's surface)
% 
%          pitch: rotates the vehicle about its y-axis relative to ENU
%          coordinates (the vehicle's orientation changes relative to
%          the Earth's surface)
% 
%          yaw: rotates the vehicle about its z-axis relative to ENU
%          coordinates (the vehicle's orientation changes relative to
%          the Earth's surface)
%      
%      sensor: ENU coordinates are transformed to this "sensor"
%              coordinates
%             
% 
%      (OPTIONAL INPUTS)
%       
% 
% OUTPUTS:
%      
%      transformed_ENUPoint_in_SensorCoord: the sensor reading from ENU
%      coordinates is transformed into the corresponding sensor
%      coordinates. 
% 
% 
% DEPENDENCIES:
% 
%      fcn_DebugTools_checkInputsToFunctions
% 
% EXAMPLES:
% 
%     See the script: script_test_fcn_Transform_ENUToSensorCoord
%     for a full test suite.

% Revision history:
%     
% 2023_06_07: sbrennan@psu.edu
% -- wrote the code for hgtransform
% 2023_06_21: Aneesh Batchu
% -- started this code (functionalized)
% -- added left GPS, right GPS and Velodyne Sensor
% 2023_06_23: Aneesh Batchu
% -- added 'allsensors' case. added roll, pitch and yaw as the input
% arguments for this function. 
% 2023_06_29: Aneesh Batchu
% -- added vehiclePose_ENU as the input to change the pose of the vehicle.

% TO DO
% 

flag_do_debug = 0;  % Flag to show the results for debugging
flag_do_plots = 0;  % % Flag to plot the final results
flag_check_inputs = 1; % Flag to perform input checking
fileID = 1; % The default file ID destination for fprintf messages

if flag_do_debug
    st = dbstack; %#ok<*UNRCH>
    fprintf(fileID,'STARTING function: %s, in file: %s\n',st(1).name,st(1).file);
end

%% check input arguments
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____                   _
%  |_   _|                 | |
%    | |  _ __  _ __  _   _| |_ ___
%    | | | '_ \| '_ \| | | | __/ __|
%   _| |_| | | | |_) | |_| | |_\__ \
%  |_____|_| |_| .__/ \__,_|\__|___/
%              | |
%              |_|
% See: http://patorjk.com/software/taag/#p=display&f=Big&t=Inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if flag_check_inputs
    % Are there the right number of inputs?
    if nargin < 3 || nargin > 3
        error('Incorrect number of input arguments')
    end
        
    % NOTE: zone types are checked below

end

%% Main code starts here
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _
%  |  \/  |     (_)
%  | \  / | __ _ _ _ __
%  | |\/| |/ _` | | '_ \
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Create a transform object connecting ground to body, body to
% sensorplatform, sensorplatform to LIDAR_sensor, sensorplatform to GPS
% sensor. Save all the handles in a structure called "handles".

figure(1);
clf;
clear handles
plot([],[]); % Create an empty plot
xlim([-15 15]);
ylim([-15 15]);

% Plot a mapping van, sensor platform bar, and LIDAR
% NOTE: dimensions here are WRONG, just approximations!!!!
% Taken from: https://www.vanguide.co.uk/guides/ford-transit-connect-dimensions-guide/


% Mapping van
length = 4.825; % Meters
width = 2.137; % Meters
height = 1.827; % Meters

% Define offsets of the cube edges relative to the cube's center. Offsets
% are added onto all points except the origin.
offset_length = length/2 - 0.878; % Meters
offset_width = 0;
tire_offset_height = 15*(1/12)*(1/3.281); % 15 inches times 1 ft is 12 inches, times 1 meter is 3.281 feet.
offset_height = height/2 - tire_offset_height;
cube_points = fcn_INTERNAL_fillCube(length,width,height,offset_length,offset_width,offset_height);
% Plot the result and make the body the parent
mapping_van_plot_handles = fcn_INTERNAL_plotCube(cube_points);

% Once the first plot is made, we can define the axes
ax = gca; % Grab current axes;
handles.transform_ground_to_body = hgtransform('Parent',ax);
handles.transform_body_to_sensorplatform = hgtransform('Parent',handles.transform_ground_to_body);
handles.transform_sensorplatform_to_LIDAR = hgtransform('Parent',handles.transform_body_to_sensorplatform);
handles.transform_sensorplatform_to_leftGPS = hgtransform('Parent',handles.transform_body_to_sensorplatform);
handles.transform_sensorplatform_to_rightGPS = hgtransform('Parent',handles.transform_body_to_sensorplatform);
handles.transform_sensorplatform_to_velodyneLIDAR = hgtransform('Parent',handles.transform_body_to_sensorplatform);
set(mapping_van_plot_handles,'Parent',handles.transform_ground_to_body); 


% Sensor platform bar
length = 2*(1/12)*(1/3.281); % inches times 1ft = 12inches times 1 meter = 3.281 feet
width = 60*(1/12)*(1/3.281); % inches times 1ft = 12inches times 1 meter = 3.281 feet
height = 2.5*(1/12)*(1/3.281); % inches times 1ft = 12inches times 1 meter = 3.281 feet
% Define offsets of the cube edges relative to the cube's center. Offsets
% are added onto all points except the origin.
offset_length = 0; % Meters
offset_width = 0;
offset_height = height/2; 
sensor_box_points = fcn_INTERNAL_fillCube(length,width,height,offset_length,offset_width,offset_height);
% Plot the result and make the sensorplatform the parent
sensor_platform_plot_handles = fcn_INTERNAL_plotCube(sensor_box_points);
set(sensor_platform_plot_handles,'Parent',handles.transform_body_to_sensorplatform);


% SICK LIDAR box
length = 10*(1/12)*(1/3.281); % inches times 1ft = 12inches times 1 meter = 3.281 feet
width = 10*(1/12)*(1/3.281); % inches times 1ft = 12inches times 1 meter = 3.281 feet
height = 10*(1/12)*(1/3.281); % inches times 1ft = 12inches times 1 meter = 3.281 feet
% Define offsets of the cube edges relative to the cube's center. Offsets
% are added onto all points except the origin.
offset_length = 0; % Meters
offset_width = 0;
offset_height = 0; 
lidar_SICK_box_points = fcn_INTERNAL_fillCube(length,width,height,offset_length,offset_width,offset_height);
% Plot the result and make the LIDAR the parent
lidar_sick_plot_handles = fcn_INTERNAL_plotCube(lidar_SICK_box_points);
set(lidar_sick_plot_handles,'Parent',handles.transform_sensorplatform_to_LIDAR);

% LeftGPS
length = 15*(1/12)*(1/3.281); % inches times 1ft = 12inches times 1 meter = 3.281 feet
width = 15*(1/12)*(1/3.281); % inches times 1ft = 12inches times 1 meter = 3.281 feet
height = 3*(1/12)*(1/3.281); % inches times 1ft = 12inches times 1 meter = 3.281 feet
% Define offsets of the cube edges relative to the cube's center. Offsets
% are added onto all points except the origin.
offset_length = 0; % Meters
offset_width = 0;
offset_height = -height/2; 
GPS_left_box_points = fcn_INTERNAL_fillCube(length,width,height,offset_length,offset_width,offset_height);
% Plot the result and make the leftGPS the parent
GPS_left_plot_handles = fcn_INTERNAL_plotCube(GPS_left_box_points);
% set(GPS_left_plot_handles,'Parent',handles.transform_GPSchannel_to_leftGPS);
set(GPS_left_plot_handles,'Parent',handles.transform_sensorplatform_to_leftGPS);

% RightGPS
length = 15*(1/12)*(1/3.281); % inches times 1ft = 12inches times 1 meter = 3.281 feet
width = 15*(1/12)*(1/3.281); % inches times 1ft = 12inches times 1 meter = 3.281 feet
height = 3*(1/12)*(1/3.281); % inches times 1ft = 12inches times 1 meter = 3.281 feet
% Define offsets of the cube edges relative to the cube's center. Offsets
% are added onto all points except the origin.
offset_length = 0; % Meters
offset_width = 0;
offset_height = -height/2; 
GPS_right_box_points = fcn_INTERNAL_fillCube(length,width,height,offset_length,offset_width,offset_height);
% Plot the result and make the RightGPS the parent
GPS_right_plot_handles = fcn_INTERNAL_plotCube(GPS_right_box_points);
% set(GPS_right_plot_handles,'Parent',handles.transform_GPSchannel_to_rightGPS);
set(GPS_right_plot_handles,'Parent',handles.transform_sensorplatform_to_rightGPS);

% Velodyne LIDAR box
length = 10*(1/12)*(1/3.281); % inches times 1ft = 12inches times 1 meter = 3.281 feet
width = 10*(1/12)*(1/3.281); % inches times 1ft = 12inches times 1 meter = 3.281 feet
height = 10*(1/12)*(1/3.281); % inches times 1ft = 12inches times 1 meter = 3.281 feet
% Define offsets of the cube edges relative to the cube's center. Offsets
% are added onto all points except the origin.
offset_length = 0; % Meters
offset_width = -(width/2)+1*(1/12)*(1/3.281);
offset_height = 0; 
lidar_velodyne_box_points = fcn_INTERNAL_fillCube(length,width,height,offset_length,offset_width,offset_height);
% Plot the result and make the LIDAR the parent
lidar_velodyne_plot_handles = fcn_INTERNAL_plotCube(lidar_velodyne_box_points);
set(lidar_velodyne_plot_handles,'Parent',handles.transform_sensorplatform_to_velodyneLIDAR);


%% Now, start moving items to their correct locations

% Move the sensor platform origin
sensor_box_offset_x_relative_to_body = -1; % Meters - GUESS!!
sensor_box_offset_y_relative_to_body =  0; % Meters - GUESS!!
sensor_box_offset_z_relative_to_body = 1.6; % Meters - GUESS!!

Mtransform_sensorplatform_translate = makehgtform('translate',[sensor_box_offset_x_relative_to_body sensor_box_offset_y_relative_to_body sensor_box_offset_z_relative_to_body]);
set(handles.transform_body_to_sensorplatform,'Matrix',Mtransform_sensorplatform_translate);


% Move the SICK LIDAR to its correct location
lidar_sick_offset_x_relative_to_sensorplatform = -0.4; % Meters - GUESS!!
lidar_sick_offset_y_relative_to_sensorplatform =  0; % Meters - GUESS!!
lidar_sick_offset_z_relative_to_sensorplatform = -0.1; % Meters - GUESS!!

sicklidar_perturbation_zrotate = 0;
sicklidar_perturbation_xrotate = 0;

Mtransform_sicklidar_translate = makehgtform('translate',[lidar_sick_offset_x_relative_to_sensorplatform, lidar_sick_offset_y_relative_to_sensorplatform, lidar_sick_offset_z_relative_to_sensorplatform]);
Mtransform_sicklidar_zrotate = makehgtform('zrotate',sicklidar_perturbation_zrotate*pi/180);
Mtransform_sicklidar_yrotate = makehgtform('yrotate',90*pi/180);
Mtransform_sicklidar_xrotate = makehgtform('xrotate',sicklidar_perturbation_xrotate*pi/180);

set(handles.transform_sensorplatform_to_LIDAR,'Matrix',Mtransform_sicklidar_translate*Mtransform_sicklidar_zrotate*Mtransform_sicklidar_yrotate*Mtransform_sicklidar_xrotate);

% Move the LEFT GPS to its correct location
gps_left_offset_x_relative_to_sensorplatform =  0.3; % Meters - GUESS!!
gps_left_offset_y_relative_to_sensorplatform =  30*(1/12)*(1/3.281); % Meters - GUESS!!
gps_left_offset_z_relative_to_sensorplatform =  0.5; % Meters - GUESS!!

Mtransform_leftgps_translate = makehgtform('translate',[gps_left_offset_x_relative_to_sensorplatform gps_left_offset_y_relative_to_sensorplatform gps_left_offset_z_relative_to_sensorplatform]);
set(handles.transform_sensorplatform_to_leftGPS,'Matrix',Mtransform_leftgps_translate);


% Move the RIGHT GPS to its correct location
gps_right_offset_x_relative_to_sensorplatform =  0.3; % Meters - GUESS!!
gps_right_offset_y_relative_to_sensorplatform =  -30*(1/12)*(1/3.281); % Meters - GUESS!!
gps_right_offset_z_relative_to_sensorplatform =  0.5; % Meters - GUESS!!

Mtransform_rightgps_translate = makehgtform('translate',[gps_right_offset_x_relative_to_sensorplatform gps_right_offset_y_relative_to_sensorplatform gps_right_offset_z_relative_to_sensorplatform]);
set(handles.transform_sensorplatform_to_rightGPS,'Matrix',Mtransform_rightgps_translate);


% Move the VELODYNE LIDAR to its correct location
lidar_velodyne_offset_x_relative_to_sensorplatform = -1.0; % Meters - GUESS!!
lidar_velodyne_offset_y_relative_to_sensorplatform =  0; % Meters - GUESS!!
lidar_velodyne_offset_z_relative_to_sensorplatform = 0.4; % Meters - GUESS!!

velodyne_perturbation_yrotate = 0;

Mtransform_velodynelidar_translate = makehgtform('translate',[lidar_velodyne_offset_x_relative_to_sensorplatform, lidar_velodyne_offset_y_relative_to_sensorplatform, lidar_velodyne_offset_z_relative_to_sensorplatform]);
Mtransform_velodynelidar_zrotate = makehgtform('zrotate',90*pi/180);
Mtransform_velodynelidar_yrotate = makehgtform('yrotate',velodyne_perturbation_yrotate*pi/180);
Mtransform_velodynelidar_xrotate = makehgtform('xrotate',-36*pi/180);

set(handles.transform_sensorplatform_to_velodyneLIDAR,'Matrix',Mtransform_velodynelidar_translate*Mtransform_velodynelidar_zrotate*Mtransform_velodynelidar_yrotate*Mtransform_velodynelidar_xrotate);


%% Set the pose of the vehicle in ENU coordinates

x = vehiclePose_ENU(1);
y = vehiclePose_ENU(2);
z = vehiclePose_ENU(3);
roll = -vehiclePose_ENU(4);
pitch = vehiclePose_ENU(5);
yaw = vehiclePose_ENU(6);

% Transform matrix for the translation
Mtranslate_vehicle = makehgtform('translate', [x y z]);

% Rotational transform matrices for yaw, pitch and roll
Mroll = makehgtform('xrotate',roll*pi/180);
Mpitch = makehgtform('yrotate',pitch*pi/180);
Myaw = makehgtform('zrotate',yaw*pi/180);

% First, translate the vehicle along x, y, and z
% Next, rotate the vehicle based on yaw, pitch and roll 
% The vehicle is rotated based on the convention
set(handles.transform_ground_to_body,'Matrix',Mtranslate_vehicle*Myaw*Mpitch*Mroll);

%% Plot the sensorReading_ENU in the ENU coordinates

plot3(sensorReading_ENU(1),sensorReading_ENU(2),sensorReading_ENU(3),'r.','MarkerSize',50);


%% Determine the sensor 

% This function determines the sensor type. The detailed description of the
% fucntion can be found in the function "fcn_Transform_determineSensor"
sensor_string = fcn_Transform_determineSensor(sensor);

%% Find the transformed_ENUPoint_in_SensorCoord


% Convert sensorReading_ENU to homogenous coordinates
sensorReading_ENU_homogenous_measurement = [sensorReading_ENU 1]';


switch lower(sensor_string)

    case 'vehicle'

        vehicle_measurement = (Mtranslate_vehicle*Myaw*Mpitch*Mroll)\sensorReading_ENU_homogenous_measurement;
        transformed_ENUPoint_in_SensorCoord = vehicle_measurement(1:3,1)';
        fprintf(fileID,'\nThe point is located, in vehicle coordinates frame, at:\n');
        disp(transformed_ENUPoint_in_SensorCoord);

    case 'sensorplatform'

        sensorplatform_measurement = (Mtransform_sensorplatform_translate*Mtranslate_vehicle*Myaw*Mpitch*Mroll)\sensorReading_ENU_homogenous_measurement;
        transformed_ENUPoint_in_SensorCoord = sensorplatform_measurement(1:3,1)';
        fprintf(fileID,'\nThe point is located, in sensorplatform frame, at:\n');
        disp(transformed_ENUPoint_in_SensorCoord);

    case 'sicklidar'

        sicklidar_measurement = (Mtransform_sensorplatform_translate*Mtransform_sicklidar_translate*Mtransform_sicklidar_yrotate*Mtranslate_vehicle*Myaw*Mpitch*Mroll)\sensorReading_ENU_homogenous_measurement;
        transformed_ENUPoint_in_SensorCoord = sicklidar_measurement(1:3,1)';
        fprintf(fileID,'\nThe point is located, in SICK lidar frame, at:\n');
        disp(transformed_ENUPoint_in_SensorCoord);

    case 'leftgps'

        leftgps_measurement = (Mtransform_sensorplatform_translate*Mtransform_leftgps_translate*Mtranslate_vehicle*Myaw*Mpitch*Mroll)\sensorReading_ENU_homogenous_measurement;
        transformed_ENUPoint_in_SensorCoord = leftgps_measurement(1:3,1)';
        fprintf(fileID,'\nThe point is located, in left GPS frame, at:\n');
        disp(transformed_ENUPoint_in_SensorCoord);

    case 'rightgps'

        rightgps_measurement = (Mtransform_sensorplatform_translate*Mtransform_rightgps_translate*Mtranslate_vehicle*Myaw*Mpitch*Mroll)\sensorReading_ENU_homogenous_measurement;
        transformed_ENUPoint_in_SensorCoord = rightgps_measurement(1:3,1)';
        fprintf(fileID,'\nThe point is located, in right GPS frame, at:\n');
        disp(transformed_ENUPoint_in_SensorCoord);

    case 'velodynelidar'

        velodynelidar_measurement = (Mtransform_sensorplatform_translate*Mtransform_velodynelidar_translate*Mtransform_velodynelidar_zrotate*Mtransform_velodynelidar_xrotate*Mtranslate_vehicle*Myaw*Mpitch*Mroll)\sensorReading_ENU_homogenous_measurement; 
        transformed_ENUPoint_in_SensorCoord = velodynelidar_measurement(1:3,1)';
        fprintf(fileID,'\nThe point is located, in Velodyne lidar frame, at:\n');
        disp(transformed_ENUPoint_in_SensorCoord);

    case 'allsensors'
        
        fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, 'vehicle');
        fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, 'sensorplatform');
        fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, 'sicklidar');
        fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, 'leftGPS');
        fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, 'rightGPS');
        fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, 'velodynelidar');


    case 'other'

        fprintf(fileID, "The sensor type is not defined yet. The sensor type will be updated soon. \n");

    otherwise

        error('Unrecognized sensor type requested: %s',sensor);

end
%% Plot the results (for debugging)?
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____       _
%  |  __ \     | |
%  | |  | | ___| |__  _   _  __ _
%  | |  | |/ _ \ '_ \| | | |/ _` |
%  | |__| |  __/ |_) | |_| | (_| |
%  |_____/ \___|_.__/ \__,_|\__, |
%                            __/ |
%                           |___/
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if flag_do_plots
    
    % Nothing to plot        
    
end

if flag_do_debug
    fprintf(fileID,'ENDING function: %s, in file: %s\n\n',st(1).name,st(1).file);
end

end

%% Functions follow
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   ______                _   _
%  |  ____|              | | (_)
%  | |__ _   _ _ __   ___| |_ _  ___  _ __  ___
%  |  __| | | | '_ \ / __| __| |/ _ \| '_ \/ __|
%  | |  | |_| | | | | (__| |_| | (_) | | | \__ \
%  |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
%
% See: https://patorjk.com/software/taag/#p=display&f=Big&t=Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%§

function cube_points = fcn_INTERNAL_fillCube(length,width,height,offset_length,offset_width, offset_height)
% fprintf(1,'Preparing cube of:\n');
% fprintf(1,'\tLength: %.2f\n',length);
% fprintf(1,'\tWidth: %.2f\n',width);
% fprintf(1,'\tHeigth: %.2f\n',height);
cube_points = [
    -length -width -height
    length -width -height
    nan     nan    nan
    -length  width -height
    length  width -height
    nan     nan    nan
    -length -width  height
    length -width  height
    nan     nan    nan
    -length  width  height
    length  width  height
    nan     nan    nan

    -length -width -height
    -length +width -height
    nan     nan    nan
    +length -width -height
    +length +width -height
    nan     nan    nan
    -length -width  height
    -length +width  height
    nan     nan    nan
    +length -width  height
    +length +width  height
    nan     nan    nan

    -length -width -height
    -length -width +height
    nan     nan    nan
    +length -width -height
    +length -width +height
    nan     nan    nan
    -length +width -height
    -length +width +height
    nan     nan    nan
    +length +width -height
    +length +width +height
    nan     nan    nan
    0 0 0
    ]*0.5;
cube_points(1:end-1,:) = cube_points(1:end-1,:)+[offset_length offset_width offset_height];
end

function plot_handles = fcn_INTERNAL_plotCube(cube_points)
% See help on hgtransform to understand why we are saving the handles
% Plot the edges
plot_handles(1) = plot3(cube_points(1:end-1,1),cube_points(1:end-1,2),cube_points(1:end-1,3),'.-','MarkerSize',20);
color_plot = get(plot_handles(1),'Color');
hold on;
axis equal;
grid on;
xlim([-15 15]);
ylim([-15 15]);

xticks(-15:1:15);
yticks(-15:1:15);
%zlim([-15 15]);
xlabel('East','FontSize',15,'FontWeight','bold');
ylabel('North','FontSize',15,'FontWeight','bold');
zlabel('Up','FontSize',15,'FontWeight','bold');

% Plot the origin
plot_handles(2) = plot3(cube_points(end,1),cube_points(end,2),cube_points(end,3),'Color',color_plot,'MarkerSize',50);

% Plot the coordinate system
most_negative_point = cube_points(1,:);
most_positive_point = cube_points(end-2,:);
differences = most_positive_point-most_negative_point;
mean_differences = mean(differences);
differences = [mean_differences mean_differences mean_differences];

% Plot x-axis
plot_handles(3) = quiver3(cube_points(end,1),cube_points(end,2),cube_points(end,3),...
    differences(1),0,0,'Color',color_plot,'LineWidth',3);
text_point = [cube_points(end,1),cube_points(end,2),cube_points(end,3)] + ...
    [differences(1),0,0];
plot_handles(4) = text(text_point(1),text_point(2),text_point(3),'X','Color',color_plot);

% Plot y-axis
plot_handles(5) = quiver3(cube_points(end,1),cube_points(end,2),cube_points(end,3),...
    0,differences(2),0,'Color',color_plot,'LineWidth',3);
text_point = [cube_points(end,1),cube_points(end,2),cube_points(end,3)] + ...
    [0,differences(2),0];
plot_handles(6) = text(text_point(1),text_point(2),text_point(3),'Y','Color',color_plot);

% Plot z-axis
plot_handles(7) = quiver3(cube_points(end,1),cube_points(end,2),cube_points(end,3),...
    0,0,differences(3),'Color',color_plot,'LineWidth',3);
text_point = [cube_points(end,1),cube_points(end,2),cube_points(end,3)] + ...
    [0,0,differences(3)];
plot_handles(8) = text(text_point(1),text_point(2),text_point(3),'Z','Color',color_plot);


end


