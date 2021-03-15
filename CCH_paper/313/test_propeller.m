Prop = PropellerKevin();
Prop.c = 0.2;
Prop.e_oswald           = 0.8;
Prop.Nb                 = 4;
Prop.Omega              = 162;
Prop.R                  = 1.3;
Prop.s                  = 0.2;
Prop.theta_t            = deg2rad(-30);
Prop.rho                = 1.2;
Prop.x_PR               = -7.66;
Prop.y_PR               = 0;
Prop.z_PR               = 0;

Prop.u = 0;
Prop.v = 0;
Prop.w = 0;
Prop.p = 0;
Prop.q = 0;
Prop.r = 0;


Prop.theta_0 = deg2rad(25);
Prop.calculate_force();
Prop.calculate_torque();