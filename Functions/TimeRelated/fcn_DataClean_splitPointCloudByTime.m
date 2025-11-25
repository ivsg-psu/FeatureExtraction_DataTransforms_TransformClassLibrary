function [dataStructure_out, Bin_Time_Start] = fcn_DataClean_splitPointCloudByTime(dataStructure, time_offset, time_decimation, varargin)

% fcn_DataClean_splitPointCloudByTime
%
% This function temporally splits raw LiDAR point cloud data into uniform
% time bins using corrected LiDAR reference timestamps. It performs alignment
% with GPS time and interpolates metadata fields to match binned structure.
%
% INPUTS:
%   LiDAR_DataStructure - struct with fields: .PointCloud (cell), .Bag_Time (Nx1), etc.
%   time_offset         - scalar, time offset between LiDAR time and GPS
%   time (GPS_Time = LiDAR_Time + time_offset)
%   time_decimation     - scalar, bin width in seconds (e.g., 0.01 or 0.1)
%   varargin
%
% OUTPUTS:
%   split_LiDAR_DataStructure - struct with point clouds split into time bins
%   Bins_timestamps           - vector of start times for each bin

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


% Does the user want to specify the GPS_Time ?
% GPS_Time = 0;
% if 3 <= nargin
%     temp = varargin{1};
%     if ~isempty(temp)
%         GPS_Time = temp;
%     end
% end



% ---- Parse options (no anonymous validators) -----------------------------
p = inputParser;
addParameter(p,'TimeField', 'Device_Time');

parse(p,varargin{:});
opt = p.Results;
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


% Step 1: Prepare raw variables

LiDAR_DataStructure = dataStructure.Lidar_Velodyne_Rear;
pointcloud_cell_array = LiDAR_DataStructure.PointCloud;
N_frames = length(pointcloud_cell_array);
TimeField = opt.TimeField;


% Get first point time offset and scan duration for each frame

first_point_time_offset = nan(N_frames,1);
last_point_time_offset = nan(N_frames,1);
for ith_frame = 1:N_frames
    if isempty(pointcloud_cell_array{ith_frame})
        continue;
    end
    points_time_offsets = pointcloud_cell_array{ith_frame}(:,5);  % 5th column is time offset
    first_point_time_offset(ith_frame) = min(points_time_offsets);
    last_point_time_offset(ith_frame) = max(points_time_offsets);
end
scan_duration  = last_point_time_offset - first_point_time_offset;


if strcmp(TimeField, 'Device_Time') || strcmp(TimeField, 'Host_Time')
    LiDAR_Time_Raw_ns = LiDAR_DataStructure.(TimeField);
    LiDAR_Time_Raw = LiDAR_Time_Raw_ns*(1e-9);
  
else
    LiDAR_Time_Ref = LiDAR_DataStructure.(TimeField);
    LiDAR_Time_Device_ns = LiDAR_DataStructure.Device_Time;
    LiDAR_Time_Device = LiDAR_Time_Device_ns*(1e-9);
    % [GPS_ROS_Time_cell_array, GPS_units] = fcn_DataClean_pullDataFromFieldAcrossAllSensors(dataStructure, TimeField, 'gps');
    % [GPS_Time_cell_array, ~] = fcn_DataClean_pullDataFromFieldAcrossAllSensors(dataStructure, "GPS_Time", 'gps');
    % N_GPS_units = length(GPS_units);
    % 
    % std_tGPS_ROS = zeros(N_GPS_units,1);
    % for idx_unit = 1:N_GPS_units
    %     tGPS = GPS_Time_cell_array{idx_unit};
    %     tGPS_ROS = GPS_ROS_Time_cell_array{idx_unit};
    %     dtGPS = tGPS_ROS - tGPS;
    %     std_tGPS_ROS(idx_unit) = std(dtGPS);
    % end
    % [~, idx_GPS_used] = min(std_tGPS_ROS);
    % tGPS = GPS_Time_cell_array{idx_GPS_used};
    % tGPS_ROS = GPS_ROS_Time_cell_array{idx_GPS_used};
    tLiDAR_trans = LiDAR_Time_Ref - LiDAR_Time_Device;
    tLiDAR_trans_med = median(tLiDAR_trans,'omitnan');

    LiDAR_Time_Raw = LiDAR_Time_Ref - scan_duration;
end
LiDAR_Time_Raw = LiDAR_Time_Raw + time_offset;

% LiDAR_Time_Adjusted = LiDAR_Time_Raw + time_offset;


startTime = min(LiDAR_Time_Raw + first_point_time_offset);
endTime = max(LiDAR_Time_Raw + last_point_time_offset);
bin_edges = (startTime:time_decimation:endTime).';
N_bins = length(bin_edges) - 1;
Bin_Time_Start = bin_edges(1:end-1);
Bin_Time_End = bin_edges(2:end);
Bin_Time_Center = (Bin_Time_Start + Bin_Time_End)/2;
Bin_Time_Eff = Bin_Time_Start;
new_scan_seq = (1:N_bins).';
split_pointcloud_cell_array = cell(N_bins,1);
split_times_cell_array  = cell(N_bins,1);
% Step 2: Assign points into time bins
validMask = isfinite(LiDAR_Time_Raw);

for ith_frame = 1:N_frames
    pointcloud_ith = pointcloud_cell_array{ith_frame};
    if isempty(pointcloud_ith) || ~validMask(ith_frame)
        continue;
    end

    % Compute absolute time of each point
    points_time_offsets = pointcloud_ith(:,5);
    point_times = LiDAR_Time_Raw(ith_frame) + points_time_offsets;

    % Only keep points that fall into bin range
    valid_mask = (point_times >= bin_edges(1)) & (point_times < bin_edges(end));
    if ~any(valid_mask)
        continue;
    end

    valid_pointcloud = pointcloud_ith(valid_mask, :);
    valid_times = point_times(valid_mask);
    
    % Assign each point to bin index
    bin_indices = floor((valid_times - startTime) / time_decimation) + 1;
    % bin_indices = floor((valid_times - (startTime - 0.5*time_decimation)) ./ time_decimation) + 1;

    % bin_indices = 
    bin_ids = unique(bin_indices);
    
    % figure(80)
    % clf;
    % scatter(valid_times,valid_times)
    % hold on
    % xline(Bin_Time_Start(bin_ids))
    % xline(Bin_Time_End(bin_ids))

    for k = 1:length(bin_ids)
        bin_id = bin_ids(k);
        if bin_id < 1 || bin_id > N_bins
            continue;
        end

        old_points = split_pointcloud_cell_array{bin_id};
        old_times = split_times_cell_array{bin_id};
        
        new_points = valid_pointcloud(bin_indices == bin_id, :);
        new_times = valid_times(bin_indices == bin_id);
        split_pointcloud_cell_array{bin_id} = [old_points; new_points];
        split_times_cell_array{bin_id} = [old_times; new_times];

    end
end

for ith_bin = 1:N_bins
    if ~isempty(split_times_cell_array{ith_bin})
        Bin_Time_Eff(ith_bin) = median(split_times_cell_array{ith_bin}, 'omitnan');
    end
end


% Step 5: Interpolate metadata fields over new bins
% -----------------------------
split_LiDAR_DataStructure = LiDAR_DataStructure;
LiDAR_fieldnames = fieldnames(split_LiDAR_DataStructure);
nonempty_mask = ~cellfun(@isempty, split_times_cell_array) & ~isnan(Bin_Time_Eff);
LiDAR_Time_Raw = LiDAR_Time_Raw(validMask);
for i = 1:length(LiDAR_fieldnames)
    field_name = LiDAR_fieldnames{i};
    field_value = split_LiDAR_DataStructure.(field_name);

    if ~iscell(field_value) && ~isempty(field_value)
        if contains(field_name, 'Time') && ~any(isnan(field_value))
            field_value = field_value(validMask,:);
            interp_value = interp1(LiDAR_Time_Raw, field_value, Bin_Time_Start, 'linear', 'extrap');
            interp_value_valid = interp_value(nonempty_mask);
            split_LiDAR_DataStructure.(field_name) = interp_value_valid;
        end
    end
end

% -----------------------------
% Step 6: Assign final outputs
% -----------------------------

split_pointcloud_cell_array = split_pointcloud_cell_array(nonempty_mask);
split_times_cell_array      = split_times_cell_array(nonempty_mask);
Bin_Time_Start = Bin_Time_Start(nonempty_mask);
Bin_Time_Eff   = Bin_Time_Eff(nonempty_mask);
new_scan_seq = new_scan_seq(nonempty_mask);
split_LiDAR_DataStructure.Seq = new_scan_seq;
split_LiDAR_DataStructure.Device_Time = Bin_Time_Start*1e9;
split_LiDAR_DataStructure.GPS_Time = Bin_Time_Eff*1e9;
split_LiDAR_DataStructure.PointCloud = split_pointcloud_cell_array;
split_LiDAR_DataStructure.Npoints = numel(Bin_Time_Start);

dataStructure_out = dataStructure;
dataStructure_out.Lidar_Velodyne_Rear = split_LiDAR_DataStructure;
end