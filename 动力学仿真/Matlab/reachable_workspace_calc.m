function [] = reachable_workspace_calc(a1,a2,a3)
%计算单足可达工作空间
theta_1=-pi/4+pi/2*rand(100000,1);
theta_2=-pi/6+pi/2*rand(100000,1);
theta_3=-3*pi/4+pi/2*rand(100000,1);

 x=a2*cos(theta_1).*cos(theta_2)-a1*sin(theta_1)+a3*cos(theta_1).*cos(theta_2+theta_3);
 x=-x;
 y=a1*cos(theta_1)+a2*sin(theta_1).*cos(theta_2)+a3*sin(theta_1).*cos(theta_2+theta_3);
 z=-a3*sin(theta_2+theta_3)-a2*sin(theta_2);
 scatter(z,x,1);


end

