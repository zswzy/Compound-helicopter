% 旋翼挥舞动力学方程推导
syms p A B C D E F G nb e mux muy l ex ey
Lbeta = [1 cos(p) sin(p);
        1 cos(p-2*pi/3) sin(p-2*pi/3); 
        1 cos(p-4*pi/3) sin(p-4*pi/3)];
Lbeta_p = diff(Lbeta,p);
Lbeta_pp = diff(Lbeta_p,p);

Lbeta_inv = inv(Lbeta);
h1 = A+2*(B*cos(p)-C*sin(p))+4*nb*(D+E*cos(p)+F*sin(p));
h2 = A+2*(B*cos(p-2*pi/3)-C*sin(p-2*pi/3))+4*nb*(D+E*cos(p-2*pi/3)+F*sin(p-2*pi/3));
h3 = A+2*(B*cos(p-4*pi/3)-C*sin(p-4*pi/3))+4*nb*(D+E*cos(p-4*pi/3)+F*sin(p-4*pi/3));

hI = [h1;h2;h3];
hM = Lbeta_inv*hI;

hM0 = [simplify(hM(1));simplify(hM(2));simplify(hM(3))];
disp('hM0:');disp(hM0);

c11 = nb*(1-e^4+4/3*(1-e^3)*(muy*cos(p)+mux*sin(p)));
c22 = nb*(1-e^4+4/3*(1-e^3)*(muy*cos(p-2*pi/3)+mux*sin(p-2*pi/3)));
c33 = nb*(1-e^4+4/3*(1-e^3)*(muy*cos(p-4*pi/3)+mux*sin(p-4*pi/3)));
CI = [c11 0 0; 0 c22 0; 0 0 c33];

d11 = l-G*(-ex*cos(p)+ey*sin(p))+nb*(4/3*(1-e^3)*(mux*cos(p)-muy*sin(p))+2*(1-e^2)*(mux*muy*cos(2*p)+1/2*(mux^2-muy^2)*sin(2*p)));
d22 = l-G*(-ex*cos(p-2*pi/3)+ey*sin(p-2*pi/3))+nb*(4/3*(1-e^3)*(mux*cos(p-2*pi/3)-muy*sin(p-2*pi/3))+2*(1-e^2)*(mux*muy*cos(2*(p-2*pi/3))+1/2*(mux^2-muy^2)*sin(2*(p-2*pi/3))));
d33 = l-G*(-ex*cos(p-4*pi/3)+ey*sin(p-4*pi/3))+nb*(4/3*(1-e^3)*(mux*cos(p-4*pi/3)-muy*sin(p-4*pi/3))+2*(1-e^2)*(mux*muy*cos(2*(p-4*pi/3))+1/2*(mux^2-muy^2)*sin(2*(p-4*pi/3))));
DI = [d11 0 0; 0 d22 0; 0 0 d33];
DM = Lbeta\(Lbeta_pp+CI*Lbeta_p+DI*Lbeta);

DM011 = int(DM(1,1),'p',0,2*pi)/(2*pi);
disp('DM011:'); disp(DM011)
DM012 = int(DM(1,2),'p',0,2*pi)/(2*pi);
disp('DM012:'); disp(DM012)
DM013 = int(DM(1,3),'p',0,2*pi)/(2*pi);
disp('DM013:'); disp(DM013)
DM021 = int(DM(2,1),'p',0,2*pi)/(2*pi);
disp('DM021:'); disp(DM021)
DM022 = int(DM(2,2),'p',0,2*pi)/(2*pi);
disp('DM022:'); disp(DM022)
DM023 = int(DM(2,3),'p',0,2*pi)/(2*pi);
disp('DM023:'); disp(DM023)
DM031 = int(DM(3,1),'p',0,2*pi)/(2*pi);
disp('DM031:'); disp(DM031)
DM032 = int(DM(3,2),'p',0,2*pi)/(2*pi);
disp('DM032:'); disp(DM032)
DM033 = int(DM(3,3),'p',0,2*pi)/(2*pi);
disp('DM033:'); disp(DM033)

DM0 = [DM011 DM012 DM013;
            DM021 DM022 DM023;
            DM031 DM032 DM033];
disp('DM0');disp(DM0);

DM0_inv = inv(DM0);
disp('DM0_inv:');disp(DM0_inv);