% A propeller version from phd thesis of Kevin
% airfoil: Clark Y
%prop(1)

theta_0 = deg2rad(65);
u = 100;
v_i = fsolve(@(x) prop(x,theta_0,u),1,optimset('Display','iter','Algorithm','Levenberg-marquardt'))
[~, X_b,~,~,~,~,~]= prop(v_i,theta_0,u)
%%
table_propeller_config = readtable('propeller_config.csv');
u = 100;
initial_theta_0 = table_propeller_config.zeros_force_theta_0(table_propeller_config.U == ceil(u));

array_theta_0           = linspace(initial_theta_0,initial_theta_0+deg2rad(20));
[~,number_of_theta_0]   = size(array_theta_0);
array_X_b               = size(array_theta_0);
array_L_b               = size(array_theta_0);
array_power             = size(array_theta_0);

v_i_last = 1;
for k = 1:number_of_theta_0
    theta_0 = array_theta_0(k);
    v_i = fsolve(@(x) prop(x,theta_0,u),v_i_last,optimset('Display','off','Algorithm','Levenberg-marquardt'));
    [~, X_b,~,~,L_b,~,~,power]= prop(v_i,theta_0,u);
    array_X_b(k)        = X_b;
    array_L_b(k)        = L_b;
    array_power(k)      = power;
    
    v_i_last = v_i;
end
figure(1)
plot(rad2deg(array_theta_0),array_X_b,'linewidth',1.5);grid on;
figure(2)
plot(rad2deg(array_theta_0),array_L_b,'linewidth',1.5);grid on;
figure(3)
plot(rad2deg(array_theta_0),array_power,'linewidth',1.5);grid on;
%% search theta so that the force is 0
%X_b = search_net_force(deg2rad(70),100);
preconfig_U = 0:10:100;
preconfig_zero_theta_0 = deg2rad([17.6 22.7 26.7 30.5 34.1 37.6 40.8 43.9 46.8 49.6 52.3]);

array_U             =(0:100)';
[number_of_U,~]     = size(array_U);
array_zero_theta_0  = zeros(number_of_U,1);

for k = 1:number_of_U
    array_U(k)
    initial_zero_theta_0 = preconfig_zero_theta_0(find(preconfig_U == 10*ceil(array_U(k)/10)));
    [zero_theta_0,~,~,~] = fsolve(@(theta_0) search_net_force(theta_0,array_U(k)), initial_zero_theta_0,optimset('Display','iter','Algorithm','levenberg-marquardt'))
    array_zero_theta_0(k)   = zero_theta_0;
end

%%
VariableNames = {'U','zeros_force_theta_0'};
table_propeller_config = array2table([array_U, array_zero_theta_0],'Variablenames',VariableNames);
writetable(table_propeller_config, 'propeller_config.csv');

function X_b = search_net_force(theta_0,w)
    [v_i,~,~,~] = fsolve(@(x) prop(x,theta_0,w),1,optimset('Display','off'));
    [~, X_b,~,~]= prop(v_i,theta_0,w);
end

%%



%%

function [Tnet, X_b,Y_b,Z_b,L_b,M_b,N_b,power]= prop(v_i,theta_0,w)
% airfoil data
array_alpha = deg2rad([-9.25	-9	-8.75	-8.5	-8.25	-8	-7.75	-7.5	-7.25	-7	-6.75	-6.5	-6.25	-6	-5.75	-5.5	-5.25	-5	-4.75	-4.5	-4.25	-4	-3.75	-3.5	-3.25	-3	-2.75	-2.5	-2.25	-2	-1.75	-1.5	-1.25	-1	-0.75	-0.5	-0.25	0	0.25	0.5	0.75	1	1.25	1.5	1.75	2	2.25	2.5	2.75	3	3.25	3.5	3.75	4	4.25	4.5	4.75	5	5.25	5.5	5.75	6	6.25	6.5	6.75	7	7.25	7.5	7.75	8	8.25	8.5	8.75	9	9.25	9.5	9.75	10	10.25	10.5	10.75	11	11.25	11.5	11.75	12	12.25	12.5	12.75	13	13.25	13.5	13.75	14	14.25	14.5	14.75	15	15.25	15.5	15.75	16	16.25	16.5	16.75	17]);
array_C_L = [-0.394	-0.4033	-0.4929	-0.4974	-0.4759	-0.4463	-0.4301	-0.4024	-0.3722	-0.3479	-0.3205	-0.2942	-0.2696	-0.2426	-0.2181	-0.1915	-0.1658	-0.1391	-0.1139	-0.0885	-0.0629	-0.0363	-0.0102	0.0167	0.0438	0.0703	0.0975	0.1243	0.151	0.1774	0.2037	0.2303	0.2571	0.2836	0.3095	0.3338	0.3551	0.376	0.4107	0.4534	0.5065	0.5439	0.5687	0.5934	0.618	0.6426	0.667	0.6912	0.7151	0.7389	0.7624	0.7857	0.809	0.8315	0.8536	0.8757	0.8987	0.922	0.9457	0.9697	0.994	1.0185	1.0426	1.0671	1.091	1.1155	1.1388	1.1627	1.1866	1.2096	1.2317	1.252	1.2691	1.2847	1.2999	1.315	1.3299	1.3447	1.3602	1.3747	1.3868	1.3917	1.3923	1.3998	1.4079	1.4163	1.4223	1.4251	1.4237	1.4287	1.4317	1.4329	1.4323	1.4303	1.4269	1.422	1.4153	1.4066	1.3959	1.3832	1.373	1.37	1.3662	1.3615	1.3564	1.351];
array_C_D = [0.08504	0.08275	0.04594	0.03456	0.02994	0.02768	0.02331	0.02155	0.02018	0.01919	0.01829	0.01724	0.01629	0.01543	0.01471	0.01408	0.01361	0.01312	0.01244	0.01171	0.01125	0.01091	0.01061	0.01036	0.01016	0.00985	0.00965	0.00942	0.00915	0.00887	0.00859	0.00838	0.0082	0.00799	0.00774	0.00732	0.00684	0.00652	0.00654	0.00663	0.00669	0.00675	0.00682	0.00691	0.00699	0.00708	0.00719	0.00732	0.00746	0.00761	0.00779	0.00799	0.0082	0.00848	0.0088	0.00914	0.00945	0.00976	0.01004	0.01032	0.01057	0.01082	0.01108	0.01132	0.0116	0.01183	0.01212	0.01238	0.01263	0.01293	0.01328	0.01375	0.01442	0.01519	0.01594	0.01659	0.01726	0.01794	0.01858	0.01929	0.02016	0.02153	0.02324	0.02452	0.02581	0.02711	0.02865	0.0305	0.03282	0.03464	0.03673	0.03907	0.04167	0.04452	0.04763	0.05103	0.05481	0.05897	0.06349	0.06838	0.07302	0.0769	0.08095	0.08517	0.08948	0.09382];

% propeller data
R       = 1.3;
Omega   = 162;
rho     = 1.2133;
s       = 0.2;
Nb      = 4;
c       = 0.2;
A       = pi*R^2;

% operation condition
u_PR    = 0;
v_PR    = 0;
w_PR    = -w;
% p       = 0;
% q       = 0;
% r       = 0;

%theta_0 = deg2rad(40);
theta_t = deg2rad(-30);

% integration
number_blade_element    = 8;
number_azimuth_element  = 4;
size_ele_R              = R/number_blade_element;
size_ele_azimuth        = 2*pi/number_azimuth_element;
array_ele_R             = linspace(0+size_ele_R/2,R-size_ele_R/2,number_blade_element);
array_ele_azimuth       = linspace(0,2*pi-size_ele_azimuth,number_azimuth_element);

X_ph = 0;
Y_ph = 0;
Z_ph = 0;
L_ph = 0;
M_ph = 0;
N_ph = 0;
for r = array_ele_R
    for psi = array_ele_azimuth
        % begin with calculating local velocity, U_P, U_T and U_inf
        U_P     = w_PR - v_i;
        U_T     = r*Omega;
        U_inf   = sqrt(U_P^2+U_T^2);
        
        % calculate the inflow angle
        phi = atan(U_P/U_T);
        
        % calculate the twist
        twist = theta_t*r/R;
        
        % calculate AoA = theta_0 + inflow angle + twist
        alpha = theta_0 + phi + twist;
        
        % calculate the local lift and drag per unit span
        C_L = interp1(array_alpha, array_C_L, alpha, 'linear','extrap');
        C_D = interp1(array_C_L, array_C_D, C_L, 'linear','extrap');
        lift = 1/2*rho*U_inf^2*c*C_L;
        drag = 1/2*rho*U_inf^2*c*C_D;

        % calculate force in blade axe per unit span
        f_ybl = drag*cos(phi) - lift*sin(phi);
        f_zbl = drag*sin(phi) + lift*cos(phi);
        
        % calculate moment in blade axe per unit span
        m_bl = -f_zbl*r;
        n_bl = f_ybl*r;
        
        % calculate force and moment in propeller hub axe per unit span
        %-------------------------------------------------------%
        % T(blade -> prop hub) = [  cos(psi)     sin(psi)   0;  %
        %                           sin(psi)    -cos(psi)   0;  %
        %                           0           0          -1]  %
        %-------------------------------------------------------%
        f_xph = sin(psi)*f_ybl;
        f_yph = -cos(psi)*f_ybl;
        f_zph = -f_zbl;
        
        l_ph = sin(psi)*m_bl;
        m_ph = -cos(psi)*m_bl;
        n_ph = -n_bl;
        
        % discret sum
        X_ph = X_ph + f_xph*size_ele_R*size_ele_azimuth;
        Y_ph = Y_ph + f_yph*size_ele_R*size_ele_azimuth;
        Z_ph = Z_ph + f_zph*size_ele_R*size_ele_azimuth;
        L_ph = L_ph + l_ph*size_ele_R*size_ele_azimuth;
        M_ph = M_ph + m_ph*size_ele_R*size_ele_azimuth;
        N_ph = N_ph + n_ph*size_ele_R*size_ele_azimuth;
        
    end
end
% calculate final force and moment in propeller hub axe
X_ph = X_ph/(2*pi)*Nb;
Y_ph = Y_ph/(2*pi)*Nb;
Z_ph = Z_ph/(2*pi)*Nb;
L_ph = L_ph/(2*pi)*Nb;
M_ph = M_ph/(2*pi)*Nb;
N_ph = N_ph/(2*pi)*Nb;

% calculate force and moment in body axe
X_b = -Z_ph;
Y_b = X_ph;
Z_b = -Y_ph;
L_b = -N_ph;
M_b = L_ph;
N_b = -M_ph;

% calculate thrust coefficient
T   = -Z_ph;
C_T = T/(rho*(Omega*R)^2*A);

% build final expression
mu = sqrt(u_PR^2+v_PR^2)/(Omega*R);
mu_z = w_PR/(Omega*R);
lambda_i = v_i/(Omega*R);
C_Tnet = C_T - lambda_i*sqrt(mu^2+(-mu_z+lambda_i)^2);
Tnet = C_Tnet*(rho*(Omega*R)^2*A);

power = abs(X_b*v_i);

end

