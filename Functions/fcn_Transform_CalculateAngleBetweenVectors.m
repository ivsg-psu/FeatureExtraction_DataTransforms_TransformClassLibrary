function [angle, angle_deg] = fcn_Transform_CalculateAngleBetweenVectors(V_1, V_2)

V_1_mag = vecnorm(V_1,2,2);
V_1_unit = V_1./V_1_mag;

V_2_mag = vecnorm(V_2,2,2);
V_2_unit = V_2./V_2_mag;

angle = (asin(vecnorm(cross(V_1_unit,V_2_unit,2),2,2)));
angle_deg = rad2deg(angle);