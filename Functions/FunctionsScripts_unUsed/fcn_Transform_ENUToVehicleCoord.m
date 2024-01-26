function transformed_ENUPoint_in_VehicleCoord = fcn_Transform_ENUToVehicleCoord(vehicleParameters, sensorPoseParameters, sensor_or_vehicle, vehiclePose_ENU, sensorReading_ENU, varargin)
% fcn_Transform_ENUToSensorCoord
%
% This function takes vehicle parameters, sensor pose parameters,
% sensor_or_vehicle string, vehiclePose_ENU as the inputs and outputs a 
% ENU point's reading (sensorReading_ENU) in sensor coordinates
%
% METHOD: 
%
% Step 1: Assign the parameters to the vehicle and the sensors
%
% Step 2: If there are any perturbations in the position or orientation of
%         the sensors, they are added to the sensors before moving them to
%         the correct locations
%
% Step 3: Set the pose of the vehicle based on the vehiclePose_ENU
%        (input)
%
% Step 4: Find the transform matrices of the sensors and vehicle based on
%         sensor_or_vehicle string by multiplying the translation and 
%         rotation transform matrices of sensor and vehicle
%
% Step 5: Find the transformed Point by multiplying the transformation
%         matrices
%         
% ASSUMPTIONS:
%
% 1) The dimension data and the location data of the vehicle and the  
%    sensors are assumed in the current function. Need to change the data
%    according to the correct information later. 
%
% 2) All the sensors are offsetted to their correct locations relative to
%    the sensor platform
%
% 3) The sensor platform is assumed to be located at the correct location
%    without any perturbation
%
% FORMAT:
%
%      transformed_ENUPoint_in_SensorCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, sensor_or_vehicle, 
%                                                   vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num)
%
% INPUTS:
%
%      vehicleParameters: The structure of all vehicle parameters
%      
%           This structure includes dimensions of the vehicle and sensors, 
%           and the offsets of all the points to locate the origin. 
%
%
%      sensorPoseParameters: The structure of all sensor pose parameters
%      
%           This structure includes the position and orientation of the 
%           sensors relative to the vehicle.
%      
%
%      sensor_or_vehicle: transform matrix of this "sensor_or_vehicle" is
%                         generated to transform the "sensor_or_vehicle" 
%                         coordinates to ENU and vice versa      
%
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
%     sensorReading_ENU: Sensor reading in ENU coordinates.
% 
% (OPTIONAL INPUTS)
%
%      perturbation_sensor: [x_Perturbation,y_Perturbation,z_Perturbation,
%                   roll_Perturbation,pitch_Perturbation,yaw_Perturbation]
%
%        Perturbation in the position of the vehicle:
% 
%           x_Perturbation: This is the perturbation of the sensor in the x
%           direction, in cm, relative to the sensor platform coordinates.
%
%           y_Perturbation: This is the perturbation of the sensor in the y
%           direction, in cm, relative to the sensor platform coordinates.
%
%           z_Perturbation: This is the perturbation of the sensor in the z
%           direction, in cm, relative to the sensor platform coordinates.
% 
%        Perturbation IN THE orientation of the vehicle - Follows ISO 
%        convention
% 
%          roll_Perturbation: The perturbation of the sensor orientation 
%          about its x-axis 
%
%          pitch_Perturbation: The perturbation of the sensor orientation 
%          about its y-axis 
% 
%          yaw_Perturbation: The perturbation of the sensor orientation 
%          about its z-axis 
%       
%
%      fig_num: The figure is plotted if fig_num is entered as the input. 
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
%      None
% 
% EXAMPLES:
% 
%     See the script: script_test_fcn_Transform_ENUToSensorCoord
%     for a full test suite.

% Revision history:
%     
% 2023_07_25: Aneesh Batchu
% -- wrote this code originally

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
    narginchk(5,7);

end

perturbation_in_sensorPose_relative_to_SensorPlatform = [0, 0, 0, 0, 0, 0];
if 6 <= nargin
    temp = varargin{1};
    if ~isempty(temp)
        perturbation_in_sensorPose_relative_to_SensorPlatform = temp; 
    end
end

% Does user want to show the plots?
fig_num = [];
if 7 == nargin
    temp = varargin{end};
    if ~isempty(temp)
        fig_num = temp;
        figure(fig_num);
        flag_do_plots = 1; 
    end
else
    if flag_do_debug
        flag_do_plots = 1;
    end
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

% This function outputs the transform matrix based on the input parameters
transform_Matrix = fcn_Transform_determineTransformMatrix_Vehicle(vehicleParameters, sensorPoseParameters, sensor_or_vehicle, vehiclePose_ENU, ...
                                                             perturbation_in_sensorPose_relative_to_SensorPlatform, fig_num);

%% Find the transformed_ENUPoint_in_SensorCoord


% Convert sensorReading_ENU to homogenous coordinates
onesColumn = ones(size(sensorReading_ENU, 1),1);
sensorReading_ENU_homogenous_measurement = [sensorReading_ENU, onesColumn]';

% fprintf(fileID,'\nThe points are located at:\n');

transformed_ENUPoint_in_VehicleCoord = zeros(size(vehiclePose_ENU,1),3);

for i = 1:size(vehiclePose_ENU,1)
    transformed_ENUPoint_in_SensorCoord_homogeneous = transform_Matrix(:,:,i)\sensorReading_ENU_homogenous_measurement(:,i);
    transformed_ENUPoint_in_VehicleCoord(i,:) = transformed_ENUPoint_in_SensorCoord_homogeneous(1:3,:)';
end

% disp(transformed_ENUPoint_in_SensorCoord);

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
    
    % Plot the sensorReading_ENU in the ENU coordinates
    plot3(sensorReading_ENU(1),sensorReading_ENU(2),sensorReading_ENU(3),'r.','MarkerSize',50);
    
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