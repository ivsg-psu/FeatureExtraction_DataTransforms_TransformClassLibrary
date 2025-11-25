function [alignedDataStructure, referenceTime] = fcn_DataClean_synchronizeGPSToLiDARTime( ...
    dataStructure, lidar_fieldname, varargin)
% fcn_DataClean_synchronizeGPSToLiDARTime
% ------------------------------------------------------------------------------
% Interpolate ALL GPS fields onto LiDAR time bins.
% Baseline time axis comes from LiDAR:
%   - If UseDeviceTime = true (default):  ref_timestamps = LiDAR.Device_Time
%   - Else:                               ref_timestamps = LiDAR.ROS_Time + TimeOffset
%
% FORMAT:
%   alignedDataStructure = fcn_SynchronizeGPS_to_LiDARTime( ...
%       dataStructure, GPS_units, GPS_Time_cell_array, lidar_fieldname, ...
%       split_LiDAR_DataStructure, ...
%       'UseDeviceTime', true, ...
%       'TimeOffset', 0, ...
%       'Debug', false);
%
% REQUIRED INPUTS:
%   dataStructure               : struct. Contains per-GPS-unit sub-structs.
%   GPS_units                   : 1xN cellstr. Names of GPS unit fields in dataStructure.
%   GPS_Time_cell_array         : 1xN cell, each is numeric vector of GPS time for that unit.
%   lidar_fieldname             : char/string. Field name for LiDAR block in aligned structure.
%   split_LiDAR_DataStructure   : struct or table-like with LiDAR time fields:
%                                 must contain 'Device_Time' and 'ROS_Time' (numeric vectors).
%
% OPTIONAL NAME-VALUE PAIRS:
%   'UseDeviceTime' (bool)      : default true. If false, use ROS_Time - TimeOffset.
%   'TimeOffset'   (double)     : default 0. Offset in seconds applied to ROS_Time
%                                 when UseDeviceTime == false.
%   'Debug'        (bool)       : default false. Verbose prints and basic sanity plots.
%
% OUTPUT:
%   alignedDataStructure        : struct mirroring dataStructure with all GPS-unit
%                                 fields temporally aligned to LiDAR time bins.
%                                 Stores LiDAR block at alignedDataStructure.(lidar_fieldname).
%
% NOTES:
%   - Time fields (name contains 'Time') are interpolated with 'linear'.
%   - Non-time numeric vectors (len > 1) are interpolated with 'pchip'.
%   - Extrapolation enabled to avoid edge NaNs. Adjust locally if undesired.
%
% This function was written on 2025_03_15 by X. Cao
% Questions or comments? xfc5113@psu.edu
% Revision history
% 2025_03_15 - X. Cao, xfc5113@psu.edu
% -- wrote the code originally

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

%% Parse inputs
p = inputParser;
addParameter(p, 'TimeField', 'Device_Time');
addParameter(p, 'TimeOffset',   0,       @(x)isnumeric(x)&&isscalar(x));
addParameter(p, 'Debug',        false,   @(x)islogical(x)&&isscalar(x));

parse(p, varargin{:});
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
%% Initialize output structure and attach LiDAR block
alignedDataStructure = dataStructure;
LiDAR_DataStructure = dataStructure.(lidar_fieldname);
alignedDataStructure.(lidar_fieldname) = LiDAR_DataStructure;

%% Derive LiDAR-based baseline time bins
% Priority: Device_Time (if UseDeviceTime), otherwise ROS_Time - TimeOffset.
TimeField = opt.TimeField;
TimeOffset = opt.TimeOffset;
% 
GPS_Time_Field = 'GPS_Time';
if strcmp(TimeField,'Device_Time')
    % LiDAR_Device_Time = double(LiDAR_DataStructure.Device_Time)*(1e-9);
    % LiDAR_GPS_Time = LiDAR_Device_Time + opt.TimeOffset;
    LiDAR_Device_Time = double(LiDAR_DataStructure.Device_Time)*(1e-9);
    LiDAR_Time = LiDAR_Device_Time;
elseif strcmp(TimeField,'GPS_Time')
    LiDAR_Time = LiDAR_DataStructure.GPS_Time;
elseif strcmp(TimeField,'ROS_Time')
    LiDAR_Time = LiDAR_DataStructure.ROS_Time;
    GPS_Time_Field = 'ROS_Time';
else
    LiDAR_Time = LiDAR_DataStructure.Bag_Time;
    GPS_Time_Field = 'Bag_Time';
end

LiDAR_GPS_Time = LiDAR_Time + TimeOffset;

if opt.Debug
    fprintf('[Sync] UseDeviceTime=%d, TimeOffset=%.9f s\n', opt.UseDeviceTime, opt.TimeOffset);
    fprintf('[Sync] LiDAR bins: N=%d, span=[%.6f, %.6f] s\n', ...
        numel(LiDAR_GPS_Time), LiDAR_GPS_Time(1), LiDAR_GPS_Time(end));
end

%% Step: Interpolate all GPS fields to LiDAR time bins
[GPS_Time_cell_array, GPS_units] = fcn_DataClean_pullDataFromFieldAcrossAllSensors(dataStructure, GPS_Time_Field, 'gps');

N_gps_units = length(GPS_units);
for idx_gps = 1:N_gps_units
    GPS_unit = GPS_units{idx_gps};
    assert(isfield(dataStructure, GPS_unit), 'Missing GPS unit field: %s', GPS_unit);
    % Source GPS times for this unit
    GPS_Time = GPS_Time_cell_array{idx_gps};

    mask_valid = isfinite(GPS_Time);
    GPS_Time = GPS_Time(mask_valid);
    % Ensure monotonic unique for interp1 base X
    [GPS_Time, sort_idx] = sort(GPS_Time(:), 'ascend');
    [GPS_Time, iuniq] = unique(GPS_Time, 'stable');
    if numel(GPS_Time) < 2
        warning('[Sync] %s: not enough valid GPS times.', GPS_unit);
        continue;
    end

    GPS_struct = dataStructure.(GPS_unit);
    GPS_struct_interp = GPS_struct;
    GPS_fields = fieldnames(GPS_struct);

    if opt.Debug
        fprintf('[Sync] GPS Unit: %s | N_src=%d -> N_unique=%d | span=[%.6f, %.6f] s\n', ...
            GPS_unit, numel(GPS_Time), numel(iuniq), GPS_Time(1), GPS_Time(end));
    end

    % Iterate each field in the GPS struct
    for idx_field = 1:length(GPS_fields)
        field_name = GPS_fields{idx_field};
        field_data = GPS_struct.(field_name);

        

        % Only numeric vectors with length > 1 are interpolated
        if ~isnumeric(field_data) || ~isvector(field_data) || numel(field_data) <= 1
            continue; 
        end
        

        field_data = field_data(mask_valid);
        field_data = field_data(sort_idx);
        field_data = field_data(iuniq);
    
        if numel(field_data) ~= numel(GPS_Time)
            warning('[Sync] Size mismatch on %s.%s; skipping.', GPS_unit, field_name);
            continue;
        end


        % Choose method: 'linear' for time-like fields, otherwise 'pchip'
        if contains(field_name, 'Time', 'IgnoreCase', true)
            method = 'linear';
            % field_data = int64(round(field_data * 1e9));
        else
            method = 'pchip';
        end

        try
            interp_vals = interp1(GPS_Time, field_data, LiDAR_GPS_Time, method, NaN);
            % Force column shape to match bins
            % interp_vals = reshape(interp_vals, size(LiDAR_GPS_Time));
       
            GPS_struct_interp.(field_name) = interp_vals;
        catch ME
            warning('[Sync] interp1 failed for %s.%s with method=%s: %s', ...
                GPS_unit, field_name, method, ME.message);
        end
   
    end

    % Write back interpolated GPS unit
    alignedDataStructure.(GPS_unit) = GPS_struct_interp;
end
referenceTime = LiDAR_GPS_Time;
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


end % main function

