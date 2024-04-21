%% Initialize
error_tolerance = 0.0001;
clc

%% Case 1 - two planes are already aligned and face to same direction

v_1 = rand(1,3);
v_2 = v_1;
R = fcn_Transform_computeTansformationBetweenTwoPlanes(v_1,v_2);
v_2_aligned = (R*v_2.').';
assert(all(v_1 - v_2_aligned <= error_tolerance))


%% Case 2 - two planes are aligned but face to opposite directions

v_1 = rand(1,3);
v_2 = -v_1;
R = fcn_Transform_computeTansformationBetweenTwoPlanes(v_1,v_2);
v_2_aligned = (R*v_2.').';

assert(all(v_1 - v_2_aligned <= error_tolerance))


%% Case 3 - two planes have random angle offset along x-axis, align plane 2 to plane 1

v_1 = rand(1,3);
angle_offset = pi*rand;
R_x = fcn_Internal_createRotationMatrix_X(angle_offset);

v_2 = (R_x*v_1.').';

R = fcn_Transform_computeTansformationBetweenTwoPlanes(v_1,v_2);
v_2_aligned = (R*v_2.').';

assert(all(v_1 - v_2_aligned <= error_tolerance))


%% Case 4 - two planes have random angle offset along x-axis, align plane 2 to plane 1

v_1 = rand(1,3);
angle_offset = pi*rand;
R_x = fcn_Internal_createRotationMatrix_X(angle_offset);

v_2 = (R_x*v_1.').';

R = fcn_Transform_computeTansformationBetweenTwoPlanes(v_2,v_1);
v_1_aligned = (R*v_1.').';

assert(all(v_2 - v_1_aligned <= error_tolerance))


%% Case 5 - two planes have 30 degree angles offset along y-axis, align plane 2 to plane 1

v_1 = rand(1,3);
angle_offset = pi*rand;
R_x = fcn_Internal_createRotationMatrix_X(pi/3);

v_2 = (R_x*v_1.').';

R = fcn_Transform_computeTansformationBetweenTwoPlanes(v_1,v_2);
v_2_aligned = (R*v_2.').';

assert(all(v_1 - v_2_aligned <= error_tolerance))




%% Functions

function R_x = fcn_Internal_createRotationMatrix_X(angle)
    R_x = [1 0 0;
           0 cos(angle) -sin(angle);
           0 sin(angle) cos(angle)];

end

function R_y = fcn_Internal_createRotationMatrix_Y(angle)
    R_y = [cos(angle) 0 sin(angle);
            0 1 0;
            -sin(angle) 0 cos(angle)];

end

function R_z = fcn_Internal_createRotationMatrix_Z(angle)
    R_z = [cos(angle) -sin(angle) 0;
            sin(angle) cos(angle) 0;
            0 0 1];

end


