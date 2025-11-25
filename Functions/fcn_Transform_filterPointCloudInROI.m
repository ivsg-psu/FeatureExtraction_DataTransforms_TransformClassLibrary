function LiDAR_Data_Output = fcn_Transform_filterPointCloudInROI(LiDAR_Data,ROI)

% fcn_Transform_filterPointCloudInROI
%
% Filters a sequence of LiDAR point clouds to retain only points within a
% specified 3D region of interest (ROI) in the vehicle coordinate frame.
%
% INPUTS:
%   LiDAR_Data - either:
%                 - a struct with field 'PointCloud' (cell array of NxM point clouds), or
%                 - a raw cell array of point clouds
%   ROI        - struct with fields:
%                 - X_lim: [xmin xmax] or N x 2 array for per-frame limits
%                 - Y_lim: [ymin ymax] or N x 2 array
%                 - Z_lim: [zmin zmax] or N x 2 array
%
% OUTPUT:
%   LiDAR_Data_Output - same format as input, but each point cloud filtered
%                       to include only points inside the ROI.

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
    error('Wrong data type')
end

X_lim = ROI.X_lim;
Y_lim = ROI.Y_lim;
Z_lim = ROI.Z_lim;
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

filtered_PointCloud_cell = fcn_Internal_filterPointInXYZ(LiDAR_PointCloud_Cell,X_lim, Y_lim, Z_lim);

if isstruct(LiDAR_Data)
    LiDAR_Data_Output = LiDAR_Data;
    LiDAR_Data_Output.PointCloud = filtered_PointCloud_cell;
elseif iscell(LiDAR_Data)
    LiDAR_Data_Output = filtered_PointCloud_cell;
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

function filtered_PointCloud_cell = fcn_Internal_filterPointInXYZ(LiDAR_PointCloud,x_limits, y_limits, z_limits)
    filtered_PointCloud_cell = {};
    raw_PointCloud_cell = LiDAR_PointCloud;
    N_scans = length(raw_PointCloud_cell);
    
   
    for idx_scan = 1:N_scans
        currentScan = raw_PointCloud_cell{idx_scan,1};
        
         if size(x_limits,1) == 1
             x_limit = x_limits;
         else
             x_limit = x_limits(idx_scan,:);
         end

         if size(y_limits,1) == 1
             y_limit = y_limits;
         else
             y_limit = y_limits(idx_scan,:);
         end

         if size(z_limits,1) == 1
             z_limit = z_limits;
         else
             z_limit = z_limits(idx_scan,:);
         end

        if ~isempty(currentScan)
    
            xyz_array = currentScan(:,1:3);
            x_valid_idxs = (xyz_array(:,1)<=max(x_limit) & (xyz_array(:,1)>=min(x_limit)));
            y_valid_idxs = (xyz_array(:,2)<=max(y_limit) & (xyz_array(:,2)>=min(y_limit)));
            z_valid_idxs = (xyz_array(:,3)<=max(z_limit) & (xyz_array(:,3)>=min(z_limit)));
            valid_idxs = x_valid_idxs&y_valid_idxs&z_valid_idxs;
            filtered_PointCloud = currentScan(valid_idxs,:);
            valid_indices_cell{idx_scan, 1} = valid_idxs;
        else
            filtered_PointCloud = [];
        end
        filtered_PointCloud_cell{idx_scan,1} = filtered_PointCloud;
    
    end
         
end