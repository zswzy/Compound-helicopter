% 推导气动力系数公式
clear all
clc

syms mux muy muz
syms p rb e 
syms a1c a1s lam0 lam1s lam1c bt0 bt0p bt1c bt1s bti btip ph qh
syms th0 th1s th1c tht
syms C %1/2*s*a0/b
syms k %delta/a0
syms nb l ex ey G

bt = bt0 + bt1c*cos(p) + bt1s*sin(p);

UT = mux*sin(p)+muy*cos(p)+rb+e;

UP = (e*a1c-bt*mux)*cos(p) + (e*a1s+bt*muy)*sin(p)...
        + muz - lam0 - e*bt0p + rb*(-bt0p+a1c*cos(p)+a1s*sin(p));
    
th = th0+th1s*sin(p)+th1c*cos(p)+(rb+e)*tht;

%%
C_Zbl = -C*int(UT^2*th+UT*UP, 'rb', 0, 1-e);

C_ZA0 = simplify(int(C_Zbl/(-C), 'p', -pi, pi)/(2*pi));

C_ZA1s = simplify(int(C_Zbl/(-C)*sin(p), 'p', -pi, pi)/pi);

C_ZA1c = simplify(int(C_Zbl/(-C)*cos(p), 'p', -pi, pi)/pi);
%%
C_Ybl = C*int(k*UT^2-UT*UP*th-UP^2, 'rb', 0, 1-e);

C_YA1s = simplify(int(C_Ybl/C*sin(p), 'p', -pi, pi)/pi);

C_YA1c = simplify(int(C_Ybl/C*cos(p), 'p', -pi, pi)/pi);

%% p22,23
C_QA = C*int((k*UT^2-UT*UP*th-UP^2)*(rb+e), 'rb', 0, 1-e);

C_QA0 = simplify(int(C_QA/C, 'p', -pi, pi)/(2*pi));

C_QA1s = simplify(int(C_QA/C*sin(p), 'p', -pi, pi)/pi);

C_QA1c = simplify(int(C_QA/C*cos(p), 'p', -pi, pi)/pi);

%% flapping
UP = bti*(-mux*cos(p)+muy*sin(p))+muz-lam0-(rb+e)*(btip+(lam1s-ph)*sin(p)+(lam1c-qh)*cos(p));
IntPart = int((UT^2*th+UT*UP)*(rb+e), 'rb', 0, 1-e);

Lb = [1 -1 cos(p) sin(p);
      1 1 cos(p+pi/2) sin(p+pi/2);
      1 -1 cos(p+2*pi/2) sin(p+2*pi/2);
      1 1 cos(p+3*pi/2) sin(p+3*pi/2)];
Lbp = [0 0 -sin(p) cos(p);
      0 0 -sin(p+pi/2) cos(p+pi/2);
      0 0 -sin(p+2*pi/2) cos(p+2*pi/2);
      0 0 -sin(p+3*pi/2) cos(p+3*pi/2)];
Lbpp = [0 0 -cos(p) -sin(p);
      0 0 -cos(p+pi/2) -sin(p+pi/2);
      0 0 -cos(p+2*pi/2) -sin(p+2*pi/2);
      0 0 -cos(p+3*pi/2) -sin(p+3*pi/2)];

CI11 = nb*(1-e^4+4/3*(1-e^3)*(muy*cos(p)+mux*sin(p)));
CI22 = nb*(1-e^4+4/3*(1-e^3)*(muy*cos(p+pi/2)+mux*sin(p+pi/2)));
CI33 = nb*(1-e^4+4/3*(1-e^3)*(muy*cos(p+pi)+mux*sin(p+pi)));
CI44 = nb*(1-e^4+4/3*(1-e^3)*(muy*cos(p+3*pi/2)+mux*sin(p+3*pi/2)));
CI = [CI11  0   0   0;
      0     CI22 0  0;
      0     0   CI33   0;
      0     0   0   CI44];

DI11 = l-G*(-ex*cos(p)+ey*sin(p))+nb*(4/3*(1-e^3)*(mux*cos(p)-muy*sin(p))+2*(1-e^2)*(mux*muy*cos(2*p)+1/2*(mux^2-muy^2)*sin(2*p)));
DI22 = l-G*(-ex*cos(p+pi/2)+ey*sin(p+pi/2))+nb*(4/3*(1-e^3)*(mux*cos(p+pi/2)-muy*sin(p+pi/2))+2*(1-e^2)*(mux*muy*cos(2*(p+pi/2))+1/2*(mux^2-muy^2)*sin(2*(p+pi/2))));
DI33 = l-G*(-ex*cos(p+pi)+ey*sin(p+pi))+nb*(4/3*(1-e^3)*(mux*cos(p+pi)-muy*sin(p+pi))+2*(1-e^2)*(mux*muy*cos(2*(p+pi))+1/2*(mux^2-muy^2)*sin(2*(p+pi))));
DI44 = l-G*(-ex*cos(p+3*pi/2)+ey*sin(p+3*pi/2))+nb*(4/3*(1-e^3)*(mux*cos(p+3*pi/2)-muy*sin(p+3*pi/2))+2*(1-e^2)*(mux*muy*cos(2*(p+3*pi/2))+1/2*(mux^2-muy^2)*sin(2*(p+3*pi/2))));
DI = [DI11  0   0   0;
      0     DI22 0  0;
      0     0   DI33   0;
      0     0   0   DI44];

DM=Lb \ (Lbpp+CI*Lbp+DI*Lb);

DM0 = simplify(int(DM, 'p', -pi,pi)/(2*pi));

%% 3桨叶旋翼
UP = bti*(-mux*cos(p)+muy*sin(p))+muz-lam0-(rb+e)*(btip+(lam1s-ph)*sin(p)+(lam1c-qh)*cos(p));
IntPart = int((UT^2*th+UT*UP)*(rb+e), 'rb', 0, 1-e);

Lb = [1 cos(p) sin(p);
      1 cos(p+2*pi/3) sin(p+2*pi/3);
      1 cos(p+4*pi/3) sin(p+4*pi/3)];

Lbp = [0 -sin(p) cos(p);
      0 -sin(p+2*pi/3) cos(p+2*pi/3);
      0 -sin(p+4*pi/3) cos(p+4*pi/3)];
  
Lbpp = [0 -cos(p) -sin(p);
      0 -cos(p+2*pi/3) -sin(p+2*pi/3);
      0 -cos(p+4*pi/3) -sin(p+4*pi/3)];
  

CI11 = nb*(1-e^4+4/3*(1-e^3)*(muy*cos(p)+mux*sin(p)));
CI22 = nb*(1-e^4+4/3*(1-e^3)*(muy*cos(p+2*pi/3)+mux*sin(p+2*pi/3)));
CI33 = nb*(1-e^4+4/3*(1-e^3)*(muy*cos(p+4*pi/3)+mux*sin(p+4*pi/3)));
CI = [CI11  0   0   ;
      0     CI22 0  ;
      0     0   CI33];

DI11 = l-G*(-ex*cos(p)+ey*sin(p))+nb*(4/3*(1-e^3)*(mux*cos(p)-muy*sin(p))+2*(1-e^2)*(mux*muy*cos(2*p)+1/2*(mux^2-muy^2)*sin(2*p)));
DI22 = l-G*(-ex*cos(p+2*pi/3)+ey*sin(p+2*pi/3))+nb*(4/3*(1-e^3)*(mux*cos(p+2*pi/3)-muy*sin(p+2*pi/3))+2*(1-e^2)*(mux*muy*cos(2*(p+2*pi/3))+1/2*(mux^2-muy^2)*sin(2*(p+2*pi/3))));
DI33 = l-G*(-ex*cos(p+4*pi/3)+ey*sin(p+4*pi/3))+nb*(4/3*(1-e^3)*(mux*cos(p+4*pi/3)-muy*sin(p+4*pi/3))+2*(1-e^2)*(mux*muy*cos(2*(p+4*pi/3))+1/2*(mux^2-muy^2)*sin(2*(p+4*pi/3))));
DI = [DI11  0   0   ;
      0     DI22 0  ;
      0     0   DI33];

DM=Lb \ (Lbpp+CI*Lbp+DI*Lb);

DM0 = simplify(int(DM, 'p', -pi,pi)/(2*pi));
