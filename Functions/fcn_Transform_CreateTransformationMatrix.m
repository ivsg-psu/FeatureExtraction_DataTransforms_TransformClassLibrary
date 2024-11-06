function Transformation_Matrix = fcn_Transform_CreateTransformationMatrix(translation, x_angle,y_angle,z_angle)
% fcn_Transform_createTransformationMatrix creates a homogeneous transformation matrix
% FORMAT:
%
%  Mtransform = fcn_Transform_CreateTransformationMatrix(translation, x_angle,y_angle,z_angle)
%
% INPUTS:
%
%      translation: a array array containing translation distance in x, y
%      and z direction. Example: [x y z]
%      
%      x_angle: rotation aound x-axis in rad
%
%      y_angle: rotation aound y-axis in rad
%
%      z_angle: rotation aound z-axis in rad
%
%
% OUTPUTS:
%
%      Transformation_Matrix: a 4x4 homogeneous transformation matrix
%
%      
% DEPENDENCIES:
%
%      fcn_DebugTools_checkInputsToFunctions
%
% EXAMPLES:   
%
% See the script: script_test_fcn_Transform_CreateTransformationMatrix
% for a full test suite.
%
% This function was written on 2024_02_08 by X.Cao
% Questions or comments? xfc5113@psu.edu

% Revision history:
% 2024_02_08 - xfc5113@psu.edu
% -- original write of the code

%% Debugging and Input checks

% Check if flag_max_speed set. This occurs if the fig_num variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.

flag_do_debug = 1;
flag_check_inputs = 1;

if flag_do_debug
    st = dbstack; %#ok<*UNRCH>
    % fprintf(1,'STARTING function: %s, in file: %s\n',st(1).name,st(1).file);
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


if flag_check_inputs
        % Are there the right number of inputs?
        narginchk(4,4);

        % Check the inputPoints input
        fcn_DebugTools_checkInputsToFunctions(...
            translation, '3column_of_numbers');
       

end
%% Compute the matrix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _       
%  |  \/  |     (_)      
%  | \  / | __ _ _ _ __  
%  | |\/| |/ _` | | '_ \ 
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Create the translation matrix
translation_matrix = makehgtform('translate',translation);
% Create the z-rotate matrix
rotation_matrix_z = makehgtform('zrotate',z_angle);
% Create the y-rotate matrix
rotation_matrix_y = makehgtform('yrotate',y_angle);
% Create the x-rotate matrix
rotation_matrix_x = makehgtform('xrotate',x_angle);
% Compute the transformation matrix
Transformation_Matrix = translation_matrix*rotation_matrix_z*...
                                rotation_matrix_y*rotation_matrix_x;
