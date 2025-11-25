function [trimmedDataStructure, t0] = fcn_DataClean_TrimByCommonGPS(dataStructure, varargin)
% fcn_DataClean_TrimByCommonGPS
% -------------------------------------------------------------------------
% Purpose:
%   Trim all GPS and LiDAR data streams so that they start from a common
%   GPS time t0. This ensures that multi-sensor data are temporally aligned
%   before further processing (e.g., motion compensation, pose estimation,
%   map generation).
%
%   The function:
%       1) Finds the earliest *valid* GPS timestamp across all GPS units.
%       2) Defines t0 as the maximum among the first valid timestamps
%          (ensures all sensors have data from t0 forward).
%       3) Trims each GPS sub-structure so that time >= t0.
%       4) (Optional) Trims LiDAR scans using Device_Time (epoch ns).
%
%   This function is part of the DataClean library and used internally by
%   several preprocessing and mapping pipelines.
%
% INPUTS:
%   dataStructure
%       A struct containing multiple GPS units and LiDAR blocks.
%       GPS units must follow the naming convention:
%           <GPSPrefix>*
%       Example default: 'GPS_1', 'GPS_2', 'GPS_3', ...
%
%   varargin - Name/value pairs:
%       'GPSPrefix'       - Prefix identifying GPS fields (default 'GPS_')
%       'GPSTimeField'    - Time-stamp field within GPS units (default 'GPS_Time')
%       'LiDARFieldName'  - LiDAR struct name (default 'Lidar_Velodyne_Rear')
%       'Inclusive'       - Keep samples where t == t0 (default true)
%       'TrimLiDAR'       - Whether to also trim LiDAR scans (default false)
%       'TimeOffset'      - Optional GPS–LiDAR time offset [sec] (default 0)
%       'Debug'           - Print debug info (default false)
%
% OUTPUTS:
%   trimmedDataStructure
%       Updated structure where each GPS unit is trimmed such that
%       GPS_Time >= t0 (or > t0 if Inclusive=false).
%       If TrimLiDAR=true, the LiDAR block is trimmed using Device_Time.
%
%   t0
%       Common alignment start time (seconds).
%       Defined as the latest among all first-valid GPS timestamps.
%
% NOTES:
%   - Assumes:
%         LiDAR.Device_Time : int64 epoch time in nanoseconds
%         GPS.*.GPS_Time    : double time in seconds
%   - LiDAR.Device_Time is converted to seconds only for masking purposes.
%   - No resampling is performed; only trimming.
%   - The function handles vectors, matrices, cell arrays, and struct arrays.
%
% This function was written by X. Cao
% Questions or comments? xfc5113@psu.edu
%
% Revision history:
%   2025_05_12: xfc5113@psu.edu
%       -- initial implementation of GPS-based trimming logic
%   2025_06_01: xfc5113@psu.edu
%       -- added support for trimming LiDAR streams
%   2025_11_24: xfc5113@psu.edu
%       -- improved heading, added detailed documentation and notes
%       -- updated parameter list and clarified assumptions
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
p = inputParser;
addParameter(p,'GPSPrefix','GPS_',@(s)ischar(s)||isstring(s));
addParameter(p,'GPSTimeField','GPS_Time',@(s)ischar(s)||isstring(s));
addParameter(p,'LiDARFieldName','Lidar_Velodyne_Rear',@(s)ischar(s)||isstring(s));
addParameter(p,'Inclusive',true,@(x)islogical(x)&&isscalar(x));
addParameter(p,'Debug',false,@(x)islogical(x)&&isscalar(x));
addParameter(p,'TimeOffset',0,@(x)isnumeric(x)&&isscalar(x));
addParameter(p,'TrimLiDAR',false,@(x)islogical(x)&&isscalar(x));
parse(p,varargin{:});
opt = p.Results;

trimmedDataStructure = dataStructure;
%% Main function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _       
%  |  \/  |     (_)      
%  | \  / | __ _ _ _ __  
%  | |\/| |/ _` | | '_ \ 
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 1) common t0 from all GPS units (seconds)
all_fields = fieldnames(dataStructure);
GPS_units = all_fields(startsWith(all_fields, string(opt.GPSPrefix)));

first_times = nan(numel(GPS_units),1);
for k = 1:numel(GPS_units)
    G = dataStructure.(GPS_units{k});
    if isfield(G,opt.GPSTimeField)
        t = G.(opt.GPSTimeField);
        if ~isempty(t)
            t = t(:);
            idx = find(isfinite(t),1,'first');
            if ~isempty(idx), first_times(k) = t(idx); end
        end
    end
end
assert(any(isfinite(first_times)), 'No valid GPS_Time found in any GPS unit.');
t0 = max(first_times(isfinite(first_times)));

if opt.Debug
    fprintf('[Trim] GPS units: %s\n', strjoin(GPS_units',', '));
    fprintf('[Trim] t0 = %.9f s\n', t0);
end

%% 2) trim all GPS units to t >= t0
for k = 1:numel(GPS_units)
    unit = GPS_units{k};
    G = dataStructure.(unit);
    if ~isfield(G,opt.GPSTimeField) || isempty(G.(opt.GPSTimeField))
        trimmedDataStructure.(unit) = G; continue;
    end
    t = G.(opt.GPSTimeField)(:);
    mask = t >= t0; if ~opt.Inclusive, mask = t > t0; end

    % time itself
    G.(opt.GPSTimeField) = t(mask);

    % any field that matches time length (numeric/cell/struct), or matrix with first dim = length(t)
    fns = fieldnames(G);
    for fi = 1:numel(fns)
        fname = fns{fi};
        if strcmp(fname,opt.GPSTimeField), continue; end
        G.(fname) = local_apply_mask(G.(fname), mask, numel(t));
    end
    trimmedDataStructure.(unit) = G;
end

%% 3) trim LiDAR using Device_Time (ns)
if opt.TrimLiDAR
    lidar_name = opt.LiDARFieldName;
    if isfield(dataStructure,lidar_name)
        L = dataStructure.(lidar_name);
        assert(isfield(L,'Device_Time') && ~isempty(L.Device_Time), ...
            'LiDAR block must contain Device_Time (ns).');
    
        dev_ns  = L.Device_Time(:);          % int64 ns
        dev_sec = double(dev_ns) * 1e-9;     % for masking only
        maskL = dev_sec >= t0; 
        if ~opt.Inclusive 
            maskL = dev_sec > t0; 
        end
    
        % Device_Time itself
        L.Device_Time = L.Device_Time(maskL);
    
        % apply same mask to any field aligned per scan (vectors/cell/struct, or matrices with first dim = N)
        N = numel(dev_ns);
        fns = fieldnames(L);
        for fi = 1:numel(fns)
            fname = fns{fi};
            if strcmp(fname,'Device_Time'), continue; end
            L.(fname) = local_apply_mask(L.(fname), maskL, N);
        end
        trimmedDataStructure.(lidar_name) = L;
    
        if opt.Debug
            fprintf('[Trim] LiDAR kept %d/%d scans (by Device_Time).\n', nnz(maskL), numel(maskL));
        end
    else
        if opt.Debug, fprintf('[Trim] LiDAR field "%s" not found. GPS trimmed only.\n', lidar_name); end
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
function v_out = local_apply_mask(v_in, mask, Nref)
% Apply boolean mask along the "scan" dimension.
% Supports:
%   - numeric/logical vectors of length Nref
%   - numeric/logical arrays with size(v,1) == Nref  -> mask first dim
%   - cell vectors of length Nref
%   - struct arrays (vector) of length Nref
%
% Otherwise returns input unchanged.

v_out = v_in;
try
    if iscell(v_in)
        if isvector(v_in) && numel(v_in)==Nref
            v_out = v_in(mask);
        elseif size(v_in,1)==Nref
            v_out = v_in(mask,:); % common for 2-D cell arrays
        end
    elseif isnumeric(v_in) || islogical(v_in)
        if isvector(v_in) && numel(v_in)==Nref
            v_out = v_in(mask);
        elseif ~isvector(v_in) && size(v_in,1)==Nref
            idx = repmat({':'},1,ndims(v_in)-1);
            v_out = v_in(mask, idx{:});
        end
    elseif isstruct(v_in)
        if isvector(v_in) && numel(v_in)==Nref
            v_out = v_in(mask);
        end
    end
catch
    % if any shape error, keep original silently
end
end
