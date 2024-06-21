function [T_correction,M_correction] = fcn_Transform_computeTansformationBetweenTwoPlanes(scan_plane, ref_plane)

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

[~, ~, ~, V_ref, ~, ~, ref_plane_distances] = fcn_geometry_fitPlaneLinearRegression(ref_plane);
[~, ~, ~, V_scan, ~, ~, ~] = fcn_geometry_fitPlaneLinearRegression(scan_plane);

T_rotation = fcn_Transform_computeRotationBetweenTwoPlanes(V_ref, V_scan);
T_rotation_obj = se3(T_rotation);
scan_plane_rotated = T_rotation_obj.transform(scan_plane);
t_trans = ref_plane - scan_plane_rotated;
t_trans_ave = mean(t_trans,1);
% [~, ~, ~, V_rotated, ~, ~, scan_plane_rotated_distances] = fcn_geometry_fitPlaneLinearRegression(scan_plane_rotated);
% ref_plane_distances_ave = mean(ref_plane_distances);
% scan_plane_rotated_distances_ave = mean(scan_plane_rotated_distances);
% dist_trans = ref_plane_distances_ave - scan_plane_rotated_distances_ave;
% 
% for idx_pts = 1:size(ref_plane,1)
%     currentPoint = ref_plane(idx_pts,:);
%     diff_pts = scan_plane_rotated - currentPoint;
%     dist_pts = vecnorm(diff_pts,2,2);
%     [~,idx_closest] = min(dist_pts);
%     corresponding_pts(idx_pts,:) = scan_plane(idx_closest,:);
%     corresponding_pts_rotated(idx_pts,:) = scan_plane_rotated(idx_closest,:);
% end
% 
% translation_dist = V_rotated.*dist_trans;
T_translate = makehgtform('translate',t_trans_ave);
% 
% % T_correction = T_translate*T_rotation;
W_array = ones(size(ref_plane,1),1);
% 
[M_correction,~,~] = fcn_Transform_FitTransformationSVD(scan_plane,ref_plane,W_array);
T_correction = T_translate*T_rotation;
% T_correction


end