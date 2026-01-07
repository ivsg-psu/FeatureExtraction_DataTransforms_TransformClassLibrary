function [LiDAR_Data_Output, M_transform_LiDAR_to_Vehicle] = fcn_Transform_transformLiDARToVehicle(LiDAR_Data,RearRightGPS_offset_relative_to_VehicleOrigin,...
                                    M_calibration_GPS_to_Vehicle, M_transform_LiDARVelodyne_to_RearRightGPS, varargin)
% fcn_Transform_transformLiDARToVehicle
%
% Transforms a sequence of LiDAR point clouds from the LiDAR frame into
% the vehicle coordinate frame using known rigid-body transformations.
%
% INPUTS:
%   LiDAR_Data                             - A struct with field 'PointCloud' (cell array), or a raw cell array of point clouds
%   RearRightGPS_offset_relative_to_VehicleOrigin
%                                          - 3x1 vector [x y z] representing the offset from the rear-right GPS to the vehicle origin (in vehicle frame)
%   M_calibration_GPS_to_Vehicle           - 4x4 homogeneous matrix: GPS → Vehicle calibration transform
%   M_transform_LiDARVelodyne_to_RearRightGPS
%                                          - 4x4 homogeneous matrix: LiDAR → rear-right GPS transform
%
% OUTPUT:
%   LiDAR_Data_Output                      - Same format as input, with each point cloud transformed into the vehicle frame


% This function was written on 2025_04_23 by X. Cao
% Questions or comments? xfc5113@psu.edu

% Revision history
% 2025_04_23 - Xinyu Cao, xfc5113@psu.edu
% -- wrote the code originally
% -----------------------------


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

if isstruct(LiDAR_Data)
    LiDAR_PointCloud_Cell = LiDAR_Data.PointCloud;
elseif iscell(LiDAR_Data)
    LiDAR_PointCloud_Cell = LiDAR_Data;
else
    error('LiDAR_Data must be a struct or cell array.');
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

%% Compute overall LiDAR → Vehicle transformation

p = inputParser;
p.addParameter('T_LV_corrected', []);
p.addParameter('flag_compute', 1);
p.parse(varargin{:});
opt = p.Results;

if opt.flag_compute
    M_transform_LiDARVelodyne_to_RearRightGPS = se3(M_transform_LiDARVelodyne_to_RearRightGPS);
    M_transform_LiDAR_to_Vehicle = fcn_Transform_CalculateTransformation_LiDARVelodyneToVehicle( ...
        RearRightGPS_offset_relative_to_VehicleOrigin, ...
        M_calibration_GPS_to_Vehicle, ...
        M_transform_LiDARVelodyne_to_RearRightGPS);
else
    M_transform_LiDAR_to_Vehicle = opt.T_LV_corrected;
end

%% Loop over all point cloud frames and apply transformation
N_frames = length(LiDAR_PointCloud_Cell);
transformed_PointCloud_Cell = cell(N_frames,1);
for ith_frame = 1:N_frames
    PointCloud_ith_frame = LiDAR_PointCloud_Cell{ith_frame,1};
    transformed_PointCloud_ith_frame = [];
    if ~isempty(PointCloud_ith_frame)
        XYZ_ith_frame_LiDAR = PointCloud_ith_frame(:,1:3);
        XYZ_ith_frame_vehicle = se3(M_transform_LiDAR_to_Vehicle).transform(XYZ_ith_frame_LiDAR);
        transformed_PointCloud_ith_frame = [XYZ_ith_frame_vehicle, PointCloud_ith_frame(:,4:end)];
    end
    transformed_PointCloud_Cell{ith_frame} = transformed_PointCloud_ith_frame;
end

%% Return in same format as input

if isstruct(LiDAR_Data)
    LiDAR_Data_Output = LiDAR_Data;
    LiDAR_Data_Output.PointCloud = transformed_PointCloud_Cell;
elseif iscell(LiDAR_Data)
    LiDAR_Data_Output = transformed_PointCloud_Cell;
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

end
