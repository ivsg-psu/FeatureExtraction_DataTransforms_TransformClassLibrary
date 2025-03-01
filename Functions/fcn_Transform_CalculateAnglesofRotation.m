function [roll, pitch, yaw] = fcn_Transform_CalculateAnglesofRotation(M_transform_Vehicle_to_ENU, varargin)
% fcn_Transform_CalculateAnglesofRotation calculates roll, pitch, and yaw
% angle with the transformation matrix
%
% FORMAT:
%
% [roll, pitch, yaw] = fcn_Transform_CalculateAnglesofRotation(Mtransform_VehicleOrigin_to_ENU, (fid), (fig_num))
%
% INPUTS:
%
%
%       Mtransform_VehicleOrigin_to_ENU: a 4x4 transformation matrix
%
%
% OPTIONAL INPUTS:
%
%      fid: the fileID where to print. Default is 1, to print results to
%      the console.
%
%      fig_num: a scalar integer value
%
% OUTPUTS:
%
%      roll: the rotation angle around x-axis
%
%      pitch: the rotation angle around y-axis
%
%      yaw: the rotation angle around z-axis
%
%
% DEPENDENCIES:
%
%      fcn_DebugTools_checkInputsToFunctions
%
%
% This function was written on 2023_10_20 by X.Cao
% Questions or comments? xfc5113@psu.edu

% Revision history:
% 2023_10_20 - wrote the code
% 2024_01_28 - added more comments, particularly to explain inputs more
% clearly


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
    narginchk(1,3);
end


% Does user want to specify fid?
fid = 0;
if 2 <= nargin
    temp = varargin{1};
    if ~isempty(temp)
        fid = temp;
    end
end


% Does user want to specify fig_num?
fig_num = -1;
if 3 <= nargin
    temp = varargin{3};
    if ~isempty(temp)
        fig_num = temp;
    end
end


if fig_num < 1
    flag_do_plots = 0;
end


if flag_do_debug
    st = dbstack; %#ok<*UNRCH>
    fprintf(1,'STARTING function: %s, in file: %s\n',st(1).name,st(1).file);
end


%% Solve for the sphere
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _       
%  |  \/  |     (_)      
%  | \  / | __ _ _ _ __  
%  | |\/| |/ _` | | '_ \ 
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Step 1: Extract the rotation matrix of the transforamtion
M_Rotation_VehicleOrigin_to_ENU = M_transform_Vehicle_to_ENU.rotm;


%% Step 2: Calculate the roll, pitch, yaw angle
roll = atan2(M_Rotation_VehicleOrigin_to_ENU(3,2), M_Rotation_VehicleOrigin_to_ENU(3,3));
pitch = atan2(-M_Rotation_VehicleOrigin_to_ENU(3,1),sqrt(M_Rotation_VehicleOrigin_to_ENU(3,2)^2+M_Rotation_VehicleOrigin_to_ENU(3,3)^2));
yaw = atan2(M_Rotation_VehicleOrigin_to_ENU(2,1),M_Rotation_VehicleOrigin_to_ENU(1,1));




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

    v_x_unit = M_transform_Vehicle_to_ENU(1:3,1).';
    v_y_unit = M_transform_Vehicle_to_ENU(1:3,2).';
    v_z_unit = M_transform_Vehicle_to_ENU(1:3,3).';
    VehicleOrigin = [0 0 0];
    % Plot the Left GPS center
    quiver3(VehicleOrigin(1,1), VehicleOrigin(1,2), VehicleOrigin(1,3), v_x_unit(1,1), v_x_unit(1,2), v_x_unit(1,3), 'Color', 'b', 'LineWidth', 1);
    hold on
    quiver3(VehicleOrigin(1,1), VehicleOrigin(1,2), VehicleOrigin(1,3), v_y_unit(1,1), v_y_unit(1,2), v_y_unit(1,3), 'Color', 'r', 'LineWidth', 1);
    quiver3(VehicleOrigin(1,1), VehicleOrigin(1,2), VehicleOrigin(1,3), v_z_unit(1,1), v_z_unit(1,2), v_z_unit(1,3), 'Color', 'g', 'LineWidth', 1);


end

if flag_do_debug
    fprintf(fileID,'ENDING function: %s, in file: %s\n\n',st(1).name,st(1).file);
end


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

