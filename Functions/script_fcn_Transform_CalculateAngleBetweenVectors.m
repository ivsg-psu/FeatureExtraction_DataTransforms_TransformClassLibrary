
%% Case 1 - Same vector

v_1 = [1 0 1];
v_2 = [1 0 1];

angle_1 = rad2deg(fcn_Transform_CalculateAngleBetweenVectors(v_1,v_2))


%% Case 2 - Opposite vectors

v_1 = [1 0 1];
v_2 = [-1 0 -1];

angle_1 = rad2deg(fcn_Transform_CalculateAngleBetweenVectors(v_1,v_2))

%% Case 3 - 60 degree vectors

v_1 = [1 0 0];
v_2 = [1 sqrt(3) 0];

angle_1 = rad2deg(fcn_Transform_CalculateAngleBetweenVectors(v_2,v_1))
%% Case 4 - 120 degree vectors
v_1 = [1 0 0];
v_2 = [-1 sqrt(3) 0];

angle_1 = rad2deg(fcn_Transform_CalculateAngleBetweenVectors(v_2,v_1))
