function M_transform_RearRightGPS_to_Vehicle = fcn_Transform_CalculateTransformation_RearRightGPSToVehicle(RearRightGPS_offset_relative_to_VehicleOrigin,M_calibration_GPS_to_Vehicle, varargin)


% fcn_Transform_CalculateTransformation_VehicleToENU
% This function takes three GPS Antenna centers, GPSFront_ENU, GPSLeft_ENU and 
% GPSRight_ENU, in ENU coordinates as a (1 x 3) vector representing 
% [x, y, z] in meters as the inputs and outputs the transformation matrix
% from vehicle coordiante system to ENU coordinate system
%
%
% FORMAT:
%
%      M_transform_Vehicle_to_ENU = fcn_Transform_CalculateTransformation_VehicleToENU(GPSFront_ENU,GPSLeft_ENU, GPSRight_ENU, RearRightGPS_offset_relative_to_VehicleOrigin, M_calibration_GPS_to_Vehicle, (fid), (fig_num))
%
% INPUTS:
% 
%      RearRightGPS_offset_relative_to_VehicleOrigin: Relative distance
%      from rear right GPS antenna to the vehicle origin
%
%      M_calibration_GPS_to_Vehicle: a 4x4 homogeneous rotation matrix from
%      rear right GPS coordiante system to the vehicle coordinate system
%
%      fid: the fileID where to print. Default is 1, to print results to
%      the console.
%

%      fig_num: a scalar integer value
%
%
% OUTPUTS:
%
%      M_transform_Vehicle_to_ENU: a transformation matrix from rear
%      right GPS coordiante system to ENU coordinate system
%
% DEPENDENCIES:
%
%
% EXAMPLES:
%
%     See the script: script_fcn_Transform_CalculateTransformation_VehicleToENU
%     for a full test suite.
%
% This function was written on 2024_11_03 by X. Cao
% Questions or comments? xfc5113@psu.edu

% Revision history
% 2024_11_03 - Xinyu Cao, xfc5113@psu.edu
% -- wrote the code originally


% To do list:
% Edit the comments
% Add comments to some new created functions

%% Debugging and Input checks

flag_do_debug = 0; % % % % Flag to plot the results for debugging
flag_check_inputs = 1; % Flag to perform input checking

% flag_do_debug = 1;

if flag_do_debug
    st = dbstack; %#ok<*UNRCH>
    fprintf(1,'STARTING function: %s, in file: %s\n',st(1).name,st(1).file);
    debug_fig_num = 999978; %#ok<NASGU>
else
    debug_fig_num = []; %#ok<NASGU>
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


if flag_check_inputs == 1
    % Are there the right number of inputs?
    narginchk(2,3);
end


% Does user want to specify fid?
fid = 0;
if 3 <= nargin
    temp = varargin{1};
    if ~isempty(temp)
        fid = temp;
    end
end


if flag_do_debug
    st = dbstack; %#ok<*UNRCH>
    fprintf(1,'STARTING function: %s, in file: %s\n',st(1).name,st(1).file);
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



%% Step 1 - Find the translational transormation matrix from vehicle origin to Rear Right GPS
% Create the translational transformation matrix from rear right GPS to vehicle
M_translation_RearRightGPS_to_Vehicle = makehgtform('translate',RearRightGPS_offset_relative_to_VehicleOrigin);

%% Step 2 - Calculate the transformation matrix from rear right GPS coordinate system to Vehicle Coordinate System  
M_rotation_RearRightGPS_to_VirtualGPS = se3(M_calibration_GPS_to_Vehicle);
M_transform_RearRightGPS_to_Vehicle = M_translation_RearRightGPS_to_Vehicle*M_rotation_RearRightGPS_to_VirtualGPS;


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

if flag_do_debug
    if fid ~= 0
        fprintf(fid,'ENDING function: %s, in file: %s\n\n',st(1).name,st(1).file);
    end
end

end