function sensorPose_Perturbation = fcn_Transform_setPerturbationToSensorPose(sensor_or_vehicle,perturbation_in_sensorPose_relative_to_SensorPlatform)
% fcn_Transform_setPerturbationToSensorPose
%
% This function takes sensor_or_vehicle string and the perturbation in
% sensor pose  as the inputs and outputs a structure of arrays of
% perturbation. Each array represents the pertubation of sensor pose for a
% different sensor
%
% METHOD: 
%
% STEP 1: Determine the "sensor" using sensor_or_vehicle string (input)
%
% STEP 2: Add perturbation to the sensor's position and orientation based 
%         on perturbation_in_sensorPose_relative_to_SensorPlatform
%
% ASSUMPTIONS:
%
% The sensor platform is assumed to be located at the correct location
% without any perturbation. Therefore, the perturbation cannot be added to
% the sensor platform's position or orientation. 
%
% FORMAT:
%
%      sensorPose_Perturbation = fcn_Transform_setPerturbationToSensorPose(sensor_or_vehicle,perturbation_in_sensorPose_relative_to_SensorPlatform)
%
% INPUTS:
%
%      sensor_or_vehicle: Perturbation is added to the position and 
%                         orientation of this "sensor_or_vehicle" 
%
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
% OUTPUTS:
%      
%      sensorPose_Perturbation: a structure of arrays. Each array
%      corresponds to the perturbation added to the sensor.
% 
% DEPENDENCIES:
% 
%     None
%
% Revision history:
%
% 2023_07_25: Aneesh Batchu
% -- wrote this code originally

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
    narginchk(2,2);
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
sensor_or_vehicle_string = fcn_Transform_determineSensorTypeOrVehicle(sensor_or_vehicle);


switch lower(sensor_or_vehicle_string)

    case 'vehicle'

        % fprintf(fileID,'\n The perturbation cannot be given to the vehicle. Choose any type of sensor. \n');

        sensorPose_Perturbation.Lidar_Sick_Rear = [0, 0, 0, 0, 0, 0];
        sensorPose_Perturbation.GPS_SparkFun_LeftRear = [0, 0, 0, 0, 0, 0];
        sensorPose_Perturbation.GPS_SparkFun_RightRear = [0, 0, 0, 0, 0, 0];
        sensorPose_Perturbation.Lidar_Velodyne_Rear = [0, 0, 0, 0, 0, 0];

    case 'gpssensorplatformrear'

        % fprintf(fileID,'\n The perturbation cannot be given to the sensor platform. The GPS Hemisphere SensorPlatform is assumed to be placed at the exact location. \n');

        sensorPose_Perturbation.Lidar_Sick_Rear = [0, 0, 0, 0, 0, 0];
        sensorPose_Perturbation.GPS_SparkFun_LeftRear = [0, 0, 0, 0, 0, 0];
        sensorPose_Perturbation.GPS_SparkFun_RightRear = [0, 0, 0, 0, 0, 0];
        sensorPose_Perturbation.Lidar_Velodyne_Rear = [0, 0, 0, 0, 0, 0];

    case 'sicklidarrear'

        % The perturbation values of sick lidar

        % sicklidar_perturbation_x = perturbation_in_sensorPose_relative_to_SensorPlatform(1); % Centimeters
        % sicklidar_perturbation_y = perturbation_in_sensorPose_relative_to_SensorPlatform(2); % Centimeters
        % sicklidar_perturbation_z = perturbation_in_sensorPose_relative_to_SensorPlatform(3); % Centimeters
        % sicklidar_perturbation_roll = perturbation_in_sensorPose_relative_to_SensorPlatform(4); % Degrees
        % sicklidar_perturbation_pitch = perturbation_in_sensorPose_relative_to_SensorPlatform(5); % Degrees
        % sicklidar_perturbation_yaw = perturbation_in_sensorPose_relative_to_SensorPlatform(6); % Degrees

        sensorPose_Perturbation.Lidar_Sick_Rear = perturbation_in_sensorPose_relative_to_SensorPlatform;
        sensorPose_Perturbation.GPS_SparkFun_LeftRear = [0, 0, 0, 0, 0, 0];
        sensorPose_Perturbation.GPS_SparkFun_RightRear = [0, 0, 0, 0, 0, 0];
        sensorPose_Perturbation.Lidar_Velodyne_Rear = [0, 0, 0, 0, 0, 0];

    case 'gpssparkfunleftrear'

        % The perturbation values of left GPS

        % leftgps_perturbation_x = perturbation_in_sensorPose_relative_to_SensorPlatform(1); % Centimeters
        % leftgps_perturbation_y = perturbation_in_sensorPose_relative_to_SensorPlatform(2); % Centimeters
        % leftgps_perturbation_z = perturbation_in_sensorPose_relative_to_SensorPlatform(3); % Centimeters
        % leftgps_perturbation_roll = perturbation_in_sensorPose_relative_to_SensorPlatform(4); % Degrees
        % leftgps_perturbation_pitch = perturbation_in_sensorPose_relative_to_SensorPlatform(5); % Degrees
        % leftgps_perturbation_yaw = perturbation_in_sensorPose_relative_to_SensorPlatform(6); % Degrees

        sensorPose_Perturbation.Lidar_Sick_Rear = [0, 0, 0, 0, 0, 0];
        sensorPose_Perturbation.GPS_SparkFun_LeftRear = perturbation_in_sensorPose_relative_to_SensorPlatform;
        sensorPose_Perturbation.GPS_SparkFun_RightRear = [0, 0, 0, 0, 0, 0];
        sensorPose_Perturbation.Lidar_Velodyne_Rear = [0, 0, 0, 0, 0, 0];

    case 'gpssparkfunrightrear'

        % The perturbation values of right GPS

        % rightgps_perturbation_x = perturbation_in_sensorPose_relative_to_SensorPlatform(1); % Centimeters
        % rightgps_perturbation_y = perturbation_in_sensorPose_relative_to_SensorPlatform(2); % Centimeters
        % rightgps_perturbation_z = perturbation_in_sensorPose_relative_to_SensorPlatform(3); % Centimeters
        % rightgps_perturbation_roll = perturbation_in_sensorPose_relative_to_SensorPlatform(4); % Degrees
        % rightgps_perturbation_pitch = perturbation_in_sensorPose_relative_to_SensorPlatform(5); % Degrees
        % rightgps_perturbation_yaw = perturbation_in_sensorPose_relative_to_SensorPlatform(6); % Degrees

        sensorPose_Perturbation.Lidar_Sick_Rear = [0, 0, 0, 0, 0, 0];
        sensorPose_Perturbation.GPS_SparkFun_LeftRear = [0, 0, 0, 0, 0, 0];
        sensorPose_Perturbation.GPS_SparkFun_RightRear = perturbation_in_sensorPose_relative_to_SensorPlatform;
        sensorPose_Perturbation.Lidar_Velodyne_Rear = [0, 0, 0, 0, 0, 0];

    case 'velodynelidarrear'

        % The perturbation values of velodyne lidar

        % velodynelidar_perturbation_x = perturbation_in_sensorPose_relative_to_SensorPlatform(1); % Centimeters
        % velodynelidar_perturbation_y = perturbation_in_sensorPose_relative_to_SensorPlatform(2); % Centimeters
        % velodynelidar_perturbation_z = perturbation_in_sensorPose_relative_to_SensorPlatform(3); % Centimeters
        % velodynelidar_perturbation_roll = perturbation_in_sensorPose_relative_to_SensorPlatform(4); % Degrees
        % velodynelidar_perturbation_pitch = perturbation_in_sensorPose_relative_to_SensorPlatform(5); % Degrees
        % velodynelidar_perturbation_yaw = perturbation_in_sensorPose_relative_to_SensorPlatform(6); % Degrees

        sensorPose_Perturbation.Lidar_Sick_Rear = [0, 0, 0, 0, 0, 0];
        sensorPose_Perturbation.GPS_SparkFun_LeftRear = [0, 0, 0, 0, 0, 0];
        sensorPose_Perturbation.GPS_SparkFun_RightRear = [0, 0, 0, 0, 0, 0];
        sensorPose_Perturbation.Lidar_Velodyne_Rear = perturbation_in_sensorPose_relative_to_SensorPlatform;

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
    
    % Plot the sensorReading_ENU in the ENU coordinates
    plot3(sensorReading_ENU(1),sensorReading_ENU(2),sensorReading_ENU(3),'r.','MarkerSize',50);
    
end

if flag_do_debug
    fprintf(fileID,'ENDING function: %s, in file: %s\n\n',st(1).name,st(1).file);
end
end
