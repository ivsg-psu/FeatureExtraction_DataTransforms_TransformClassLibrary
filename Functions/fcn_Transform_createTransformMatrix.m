function Mtransform_sensorMount_to_ENU = fcn_Transform_createTransformMatrix(sensorMount_PoseENU)


Mtransform_sensorMount_to_ENU_translation = makehgtform('translate',sensorMount_PoseENU(:,1:3));

Mtransform_sensorMount_to_ENU_zrotate = makehgtform('zrotate',sensorMount_PoseENU(:,6));
Mtransform_sensorMount_to_ENU_yrotate = makehgtform('yrotate',sensorMount_PoseENU(:,5));
Mtransform_sensorMount_to_ENU_xrotate = makehgtform('xrotate',sensorMount_PoseENU(:,4));

Mtransform_sensorMount_to_ENU = Mtransform_sensorMount_to_ENU_translation*Mtransform_sensorMount_to_ENU_zrotate*...
                                Mtransform_sensorMount_to_ENU_yrotate*Mtransform_sensorMount_to_ENU_xrotate;
