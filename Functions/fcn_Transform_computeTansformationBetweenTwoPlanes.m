function R = fcn_Transform_computeTansformationBetweenTwoPlanes(V_1, V_2)

% fcn_Transform_computeTansformationBetweenTwoPlanes computes the
% rotation matrix that aligned plane 2 two plane 1 with given normal
% vectors
%
% FORMAT:
%
% R = fcn_Transform_computeTansformationBetweenTwoPlanes(V_1, V_2)
%
% INPUTS:
%
%      V_1: normal vector of plane 1, which is the reference plane 
%
%      V_2: normal vector of plane 2, which is the plane need to be aligned
%
%      (OPTIONAL INPUTS)
%
%      fig_num: a figure number to plot results. If set to -1, skips any
%      input checking or debugging, no figures will be generated, and sets
%      up code to maximize speed.
%
% OUTPUTS:
%
%      R: the rotation matirx that aligned plane 2 to plane 1
%
%
% DEPENDENCIES:
%
%      fcn_DebugTools_checkInputsToFunctions
%      fcn_geometry_plotCircle
%
% EXAMPLES:
%      
%      % BASIC example
%      points = [0 0; 1 4; 0.5 -1];
%      [centers,radii] = fcn_geometry_circleCenterFrom3Points(points,1)
% 
% See the script: script_test_fcn_Transform_CalculateAngleBetweenVectors
% for a full test suite.
%
% This function was written on 2023_10_20 by X.Cao
% Questions or comments? xfc5113@psu.edu

% Revision history:
% 2024_04_18 - wrote the code
% 2024_01_28 - added more comments, particularly to explain inputs more
% clearly


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
flag_check_inputs = 1;
if flag_check_inputs
    % Are there the right number of inputs?
    narginchk(2,2);

end

% Does user want to show the plots?
% if 6 == nargin
%     temp = varargin{end};
%     if ~isempty(temp)
%         fig_num = temp;
%         figure(fig_num);
%         flag_do_plots = 1; 
%     end
% else
%     if flag_do_debug
%         flag_do_plots = 1;
%     end
% end

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


% Normalize the first vector, find the unit vector
V_1_mag = vecnorm(V_1,2,2);
V_1_unit = V_1./V_1_mag;
% Normalize the second vector, find the unit vector
V_2_mag = vecnorm(V_2,2,2);
V_2_unit = V_2./V_2_mag;

angle_rot = fcn_Transform_CalculateAngleBetweenVectors(V_1_unit, V_2_unit);

if angle_rot == 0 % two planes are aligned and same direction
    R = eye(3);
elseif angle_rot == pi % two planes are aligned and opposite direction
    R = -eye(3);
else
    vec_rot = cross(V_2_unit,V_1_unit);
    vec_rot_mag = vecnorm(vec_rot,2,2);
    vec_rot_unit = vec_rot./vec_rot_mag;
    vec_rot_x = vec_rot_unit(1);
    vec_rot_y = vec_rot_unit(2);
    vec_rot_z = vec_rot_unit(3);
    K = [0 -vec_rot_z vec_rot_y;
        vec_rot_z, 0, -vec_rot_x;
        -vec_rot_y, vec_rot_x, 0];
    I_3 = eye(3);

    R = I_3+sin(angle_rot)*K+(1-cos(angle_rot))*K^2;
end




end