function Mtransform_ENU_to_RearGPSCenter = fcn_Transform_CalculateTransformation_RearGPSCenter(sensorMount_PoseENU)



Mtransform_RearGPSCenter_to_ENU_translation = makehgtform('translate',sensorMount_PoseENU(1:3));

Mtransform_RearGPSCenter_to_ENU_zrotate = makehgtform('zrotate',sensorMount_PoseENU(6));
Mtransform_RearGPSCenter_to_ENU_yrotate = makehgtform('yrotate',sensorMount_PoseENU(5));
Mtransform_RearGPSCenter_to_ENU_xrotate = makehgtform('xrotate',sensorMount_PoseENU(4));
Mtransform_ENU_to_RearGPSCenter = Mtransform_RearGPSCenter_to_ENU_translation*Mtransform_RearGPSCenter_to_ENU_zrotate*...
                                Mtransform_RearGPSCenter_to_ENU_yrotate*Mtransform_RearGPSCenter_to_ENU_xrotate;
