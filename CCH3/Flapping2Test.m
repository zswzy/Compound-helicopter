a0_list = zeros(1,100);
a1s_list = zeros(1,100);
a1c_list = zeros(1,100);

iu=1;
for u = 1:1:100
    [a0,a1s,a1c] = Flapping2(u,0,0,deg2rad(12),0,0,10);
    a0_list(iu) = a0;
    a1s_list(iu) = a1s;
    a1c_list(iu) = a1c;
    
    iu=iu+1;
end
figure(1)
plot(1:1:100,rad2deg(a0_list),'linewidth',2)
xlabel('u');ylabel('a0(deg)')

figure(2)
plot(1:1:100,rad2deg(a1s_list),'linewidth',2)
hold on
plot(1:1:100,rad2deg(a1c_list),'linewidth',2)
hold off
xlabel('u');ylabel('deg')
legend('a1s','a1c')