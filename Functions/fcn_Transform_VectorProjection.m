function v_projection = fcn_Transform_VectorProjection(v, u)

u_mag = vecnorm(u,2,2);

v_projection = dot(v,u,2)./(u_mag.^2).*u;
