function [VehiclePose,M_transform_Vehicle_to_ENU_matrix] = fcn_Transform_estimateVehiclePoseinENU(GPSFront_ENU_array, GPSLeft_ENU_array, GPSRight_ENU_array, varargin)
% fcn_Transform_findVehiclePoseinENU
%
% This function takes two GPS Antenna centers, GPSLeft_ENU and 
% GPSRight_ENU, in ENU coordinates as a (1 x 3) vector representing 
% [x, y, z] in meters, PITCH_vehicle_ENU in degrees and the sensor 
% mount's offset relative to the vehicle's origin as
% 
% FORMAT:
%
%     [VehiclePose,M_transform_VehicleOrigin_to_ENU_matrix] = fcn_Transform_findVehiclePoseinENU(GPSFront_ENU,GPSLeft_ENU, GPSRight_ENU, (RearRightGPS_offset_relative_to_VehicleOrigin), (M_calibration_GPS_to_Vehicle),(fid), (fig_num))
%
% INPUTS:
%
%      GPSFront_ENU: a 1x3 array contains the ENU coordinates for front GPS
%
%      GPSLeft_ENU: a 1x3 array contains the ENU coordinates for rear left GPS
%
%      GPSRight_ENU: a 1x3 array contains the ENU coordinates for rear right GPS
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
%      ref_baseStationLLA: the LLA coordinates of the reference base
%      station
%
%
% OUTPUTS:
%
%      VehiclePose: a Nx6 array contains vehicle pose, format: [x, y, z,
%      roll, pitch, yaw]
%
%      M_transform_Vehicle_to_ENU: a transformation matrix from rear
%      right GPS coordiante system to ENU coordinate system
%
% DEPENDENCIES:
%
%
% EXAMPLES:
%
%     See the script: script_fcn_Transform_estimateVehiclePoseinENU
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
    narginchk(3,8);
end


% Does user want to specify RearRightGPS_offset_relative_to_VehicleOrigin?
RearRightGPS_offset_relative_to_VehicleOrigin = load("Data\RearRightGPS_offset_relative_to_VehicleOrigin.mat");;
if 4 <= nargin
    temp = varargin{1};
    if ~isempty(temp)
        RearRightGPS_offset_relative_to_VehicleOrigin = temp;
    end
end

if isstruct(RearRightGPS_offset_relative_to_VehicleOrigin)
    fields = fieldnames(RearRightGPS_offset_relative_to_VehicleOrigin);
    actualName = fields{1};
    RearRightGPS_offset_relative_to_VehicleOrigin = RearRightGPS_offset_relative_to_VehicleOrigin.(actualName);
end


% Does user want to specify RearRightGPS_offset_relative_to_VehicleOrigin?
M_calibration_GPS_to_Vehicle = load("Data\Rotation_GPS2Vehicle_2024-05-15.mat");
if 5 <= nargin
    temp = varargin{2};
    if ~isempty(temp)
        M_calibration_GPS_to_Vehicle = temp;
    end
end

if isstruct(M_calibration_GPS_to_Vehicle)
    fields = fieldnames(M_calibration_GPS_to_Vehicle);
    actualName = fields{1};
    M_calibration_GPS_to_Vehicle = M_calibration_GPS_to_Vehicle.(actualName);
end

% Does user want to specify fid?
fid = 0;
if 6 <= nargin
    temp = varargin{3};
    if ~isempty(temp)
        fid = temp;
    end
end

% Does user want to specify fig_num?
fig_num = -1;
flag_do_plots = 0;
if 7 <= nargin
    temp = varargin{4};
    if ~isempty(temp)
        fig_num = temp;
        flag_do_plots = 1;
    end
end

% Does user want to specify ref_baseStationLLA?
ref_baseStationLLA = [40.86368573 -77.83592832 344.189];
if 8 <= nargin
    temp = varargin{5};
    if ~isempty(temp)
        fig_num = temp;
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

N_points = size(GPSLeft_ENU_array,1);
M_transform_Vehicle_to_ENU_matrix = nan(4,4,N_points);
VehiclePose = nan(N_points,6);
for idx_point = 1:N_points
    GPSLeft_ENU = GPSLeft_ENU_array(idx_point,:);
    GPSRight_ENU = GPSRight_ENU_array(idx_point,:);
    GPSFront_ENU = GPSFront_ENU_array(idx_point,:);
    if all([~isnan(GPSLeft_ENU) ~isnan(GPSFront_ENU) ~isnan(GPSRight_ENU)])
        M_transform_Vehicle_to_ENU = fcn_Transform_CalculateTransformation_VehicleToENU(GPSFront_ENU, GPSLeft_ENU, GPSRight_ENU,RearRightGPS_offset_relative_to_VehicleOrigin,M_calibration_GPS_to_Vehicle);
   
        [roll,pitch,yaw] = fcn_Transform_CalculateAnglesofRotation(M_transform_Vehicle_to_ENU);
        M_transform_Vehicleto_ENU_obj = se3(M_transform_Vehicle_to_ENU);
        VehiclePose_ENU = M_transform_Vehicleto_ENU_obj.transform([0 0 0]);
        VehiclePose(idx_point,:) = [VehiclePose_ENU,roll,pitch,yaw];
        M_transform_Vehicle_to_ENU_matrix(:,:,idx_point) = M_transform_Vehicle_to_ENU.tform;
    end
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
    figure(fig_num)
    VehiclePose_ENU_array = VehiclePose(:,1:3);
    VehiclePose_LLA_array = enu2lla(VehiclePose_ENU_array,ref_baseStationLLA,'ellipsoid');
    VehiclePose_latitude = VehiclePose_LLA_array(:,1);
    VehiclePose_longitude = VehiclePose_LLA_array(:,2);
    geoscatter(VehiclePose_latitude,VehiclePose_longitude,40,'b','filled')
    geobasemap satellite

end

if flag_do_debug
    if fid~=0
        fprintf(fid,'ENDING function: %s, in file: %s\n\n',st(1).name,st(1).file);
    end
end

end