function [LiDAR_Data_Output,VehiclePose] = fcn_Transform_transformVehicleToENU(LiDAR_Data,GPS_Front_ENU_array,...
                                                                 GPS_LeftRear_ENU_array, GPS_RightRear_ENU_array,RearRightGPS_offset_relative_to_VehicleOrigin,M_calibration_GPS_to_Vehicle)
% fcn_Transform_transformVehicleToENU
% -------------------------------------------------------------------------
% Purpose:
%   Transform LiDAR point clouds from the Vehicle coordinate frame into the
%   global ENU frame on a per-scan basis. This function:
%
%   1) Estimates the vehicle pose in ENU coordinates for each LiDAR frame
%      using three GPS antennas (front, left-rear, right-rear) and the
%      known GPS-to-Vehicle calibration matrix.
%
%   2) Uses the resulting vehicle-to-ENU transformation to convert each
%      LiDAR scan from Vehicle coordinates to ENU coordinates.
%
%   3) Returns both the transformed LiDAR point clouds and the estimated
%      vehicle poses.
%
% INPUTS:
%   LiDAR_Data
%       Either:
%           (a) a struct with field .PointCloud = {N_frames x 1} cell array
%           (b) a raw cell array of point clouds {N_frames x 1}
%       Each scan is an M x K matrix:
%           [:,1:3] = XYZ in Vehicle frame
%           [:,4:K] = additional channels (intensity, ring, etc.)
%
%   GPS_Front_ENU_array
%   GPS_LeftRear_ENU_array
%   GPS_RightRear_ENU_array
%       ENU positions (in meters) from each GPS antenna.
%       Each is N_frames x 3: [xEast, yNorth, zUp].
%
%   RearRightGPS_offset_relative_to_VehicleOrigin
%       1x3 vector describing the rear-right GPS antenna position in the
%       Vehicle coordinate frame.
%
%   M_calibration_GPS_to_Vehicle
%       4x4 homogeneous transform converting GPS frame → Vehicle frame.
%
% OUTPUTS:
%   LiDAR_Data_Output
%       Same format as input LiDAR_Data, but with all point clouds
%       converted into the ENU frame.
%
%   VehiclePose
%       N_frames x 3 array:
%           [xEast, yNorth, yawENU]
%       Output from fcn_Transform_estimateVehiclePoseinENU.
%
% NOTES:
%   - The function first calls:
%         fcn_Transform_estimateVehiclePoseinENU
%     to obtain:
%         VehiclePose
%         M_transform_Vehicle_to_ENU_matrix  (cell array of SE(3) matrices)
%
%   - Each LiDAR frame is then transformed via:
%         XYZ_ENU = se3(M_transform_Vehicle_to_ENU).transform(XYZ_vehicle)
%
%   - If an input LiDAR frame is empty or if its corresponding VehiclePose
%     contains NaNs, that frame is skipped.
%
%   - Additional LiDAR channels (e.g., intensity) are preserved.
%
% This function was written by X. Cao
% Questions or comments? xfc5113@psu.edu
%
% Revision history:
%   2025_06_04: xfc5113@psu.edu
%       -- initial implementation of vehicle-to-ENU LiDAR transformation
%   2025_08_12: xfc5113@psu.edu
%       -- updated to use external pose-estimation function
%   2025_11_24: xfc5113@psu.edu
%       -- standardized documentation and refined input handling

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


%% 0) Extract LiDAR point-cloud cell array
% Accept either:
%   - a struct with .PointCloud field, or
%   - a cell array of point clouds directly.
if isstruct(LiDAR_Data)
    LiDAR_PointCloud_Cell = LiDAR_Data.PointCloud;
elseif iscell(LiDAR_Data)
    LiDAR_PointCloud_Cell = LiDAR_Data;
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

% Number of LiDAR frames / scans
N_frames = length(LiDAR_PointCloud_Cell);

% Preallocate cell array for ENU-frame point clouds
transformed_PointCloud_ENU_Cell = cell(N_frames,1);

%% 1) Estimate vehicle pose and per-frame transforms in ENU
% This function returns:
%   VehiclePose: [xEast, yNorth, yaw] per frame
%   M_transform_Vehicle_to_ENU_matrix: per-frame SE(3) transforms
[VehiclePose, M_transform_Vehicle_to_ENU_matrix] = fcn_Transform_estimateVehiclePoseinENU( ...
    GPS_Front_ENU_array, GPS_LeftRear_ENU_array, GPS_RightRear_ENU_array, ...
    RearRightGPS_offset_relative_to_VehicleOrigin, M_calibration_GPS_to_Vehicle);

%% 2) Transform each LiDAR frame from Vehicle to ENU
for ith_frame = 1:N_frames
    % Extract current LiDAR scan (Vehicle frame)
    PointCloud_ith_scan = LiDAR_PointCloud_Cell{ith_frame,1};

    % Only process if the scan is non-empty and the pose is valid
    if ~isempty(PointCloud_ith_scan) && all(~isnan(VehiclePose(ith_frame,:)))
        % ENU positions from GPS (currently not used in the transform call,
        % but kept here for clarity / debugging if needed)
        GPSFront_ENU = GPS_Front_ENU_array(ith_frame,:); %#ok<NASGU>
        GPSLeft_ENU  = GPS_LeftRear_ENU_array(ith_frame,:); %#ok<NASGU>
        GPSRight_ENU = GPS_RightRear_ENU_array(ith_frame,:); %#ok<NASGU>

        % Per-frame vehicle-to-ENU homogeneous transform
        % (stored as a cell array of 4x4 matrices)
        M_transform_Vehicle_to_ENU = M_transform_Vehicle_to_ENU_matrix{ith_frame};

        % Extract XYZ in Vehicle frame
        XYZ_kth_scan_Vehicle = PointCloud_ith_scan(:,1:3);

        % Apply SE(3) transform to obtain ENU coordinates
        XYZ_kth_scan_ENU = se3(M_transform_Vehicle_to_ENU).transform(XYZ_kth_scan_Vehicle);

        % Reattach additional channels (e.g., intensity, ring index)
        PointCloud_ENU_kth_scan = [XYZ_kth_scan_ENU, PointCloud_ith_scan(:,4:end)];

        % Store in output cell array
        transformed_PointCloud_ENU_Cell{ith_frame,1} = PointCloud_ENU_kth_scan;
    end
end

%% 3) Package output in same format as input
% If the input was a struct, return a struct with updated .PointCloud;
% if the input was a cell array, return the transformed cell array.
if isstruct(LiDAR_Data)
    LiDAR_Data_Output = LiDAR_Data;
    LiDAR_Data_Output.PointCloud = transformed_PointCloud_ENU_Cell;
elseif iscell(LiDAR_Data)
    LiDAR_Data_Output = transformed_PointCloud_ENU_Cell;
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
