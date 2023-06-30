function sensor = fcn_Transform_determineSensor(type_of_sensor)
% fcn_Transform_determineSensor
% This function determines the sensor based on user inputs
%
% FORMAT:
%
%      sensor = fcn_Transform_determineSensor(type_of_sensor)
%
% INPUTS:
%
%      type_of_sensor: The name of the sensor
%
%      (OPTIONAL INPUTS)
%
%      (none)
%
% OUTPUTS:
%
%      sensor: a string listing the data type, one of:
%             'sick', 'velodyne', 'rightGPS', 'leftGPS', 'sensorplatform',
%             'vehicle', 'all'.
%
%      if the sensor is not recognized, it lists 'other'.
%
% DEPENDENCIES:
%
%      fcn_DebugTools_checkInputsToFunctions
%
% EXAMPLES:
%
%     See the script: script_test_fcn_Transform_determineSensor
%     for a full test suite.
%
% This function was written on 2023_06_21 by A. Batchu

% Revision history
% 

% TO DO
% 

flag_do_debug = 0;  % Flag to show the results for debugging
flag_do_plots = 0;  % % Flag to plot the final results
flag_check_inputs = 1; % Flag to perform input checking

if flag_do_debug
    st = dbstack; %#ok<*UNRCH>
    fprintf(1,'STARTING function: %s, in file: %s\n',st(1).name,st(1).file);
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
    if nargin < 1 || nargin > 1
        error('Incorrect number of input arguments')
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


type_of_sensor_lower = lower(type_of_sensor);
if contains(type_of_sensor_lower,'sick')
    sensor = 'sicklidar';
elseif contains(type_of_sensor_lower,'velodyne')
    sensor = 'velodynelidar';
elseif contains(type_of_sensor_lower,'right') && contains(type_of_sensor_lower,'gps')
    sensor = 'rightGPS';
elseif contains(type_of_sensor_lower,'left') && contains(type_of_sensor_lower,'gps')
    sensor = 'leftGPS';
elseif contains(type_of_sensor_lower,'sensor') && contains(type_of_sensor_lower,'platform')
    sensor = 'sensorplatform';
elseif contains(type_of_sensor_lower,'vehicle')
    sensor = 'vehicle';
elseif contains(type_of_sensor_lower,'all')
    sensor = 'allsensors';
else
    sensor = 'other';
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
    fprintf(1,'ENDING function: %s, in file: %s\n\n',st(1).name,st(1).file);
end

end % Ends main function




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
