% 2.12
clear all
clc

Prop = PropellerNUAA();

Prop.x_H        = -7.66;
Prop.y_H        = 0;
Prop.z_H        = 0;

Prop.theta_0        = deg2rad(15);
Prop.calculate_force();