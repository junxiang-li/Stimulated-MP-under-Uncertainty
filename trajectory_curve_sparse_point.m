function [x_temp,y_temp,k_temp,k_deviation_temp]=trajectory_curve_sparse_point(parameter,state_initial)

%global s_step
global number
global rotate_theta
global plane_phase

s_step=0.01;
x(1)=state_initial(1);
y(1)=state_initial(2);
theta(1)=state_initial(3);
k_curve(1)=state_initial(4);
s(1)=0;

steps=floor(abs(parameter(4))/s_step);
for i=2:steps
    s(i)=s(i-1)+s_step;
    k_curve(i)=k_curve(1)+parameter(1)*s(i)+parameter(2)*s(i)^2+parameter(3)*s(i)^3;
    k_curve_deviation(i)=parameter(1)+parameter(2)*s(i)+parameter(3)*s(i)^2;
    theta(i)=theta(i-1)+k_curve(i)*s_step;
    x(i)=x(i-1)+cos(theta(i))*s_step;
    y(i)=y(i-1)+sin(theta(i))*s_step;
end
state_terminal=[x(steps), y(steps),theta(steps),k_curve(steps)];

% plot(s,theta,'g',s,k_curve,'b',s,k_curve_deviation,'r');
% hold on;

if rotate_theta~=0
    if plane_phase==1 %
        origin(1:length(x),1)=x;
        origin(1:length(x),2)=y;
    else
        origin(1:length(x),1)=x;
        origin(1:length(x),2)=-y;
    end
    shift=[0,0];
    rotate_theta_temp=-rotate_theta;
    [x_transform,y_transform]=coordinate_transform(origin,rotate_theta_temp,shift);
    interval=floor(steps/number);%calculate the final point of the interval
    for i=1:number
        x_temp(i)=x_transform(i*interval);
        y_temp(i)=y_transform(i*interval);
        k_temp(i)=k_curve(i*interval);
        k_deviation_temp(i)= k_curve_deviation(i*interval);
    end
else
    %  number=floor(steps/10);
    interval=floor(steps/number);%calculate the final point of the interval
    for i=1:number
        x_temp(i)=x(i*interval);
        y_temp(i)=y(i*interval);
        k_temp(i)=k_curve(i*interval);
        k_deviation_temp(i)= k_curve_deviation(i*interval);
    end
    %     if plane_phase~=1
    %         y_temp=-y_temp;
    %     end
end
% plot(x_temp,y_temp);
% grid on;
% axis([-5 5 -5 5]);
% axis equal;
% hold on;



