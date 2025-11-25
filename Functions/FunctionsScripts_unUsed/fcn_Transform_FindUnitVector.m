function V_unit = fcn_Transform_FindUnitVector(V)

V_mag = vecnorm(V,2,2);
V_unit = V./V_mag;