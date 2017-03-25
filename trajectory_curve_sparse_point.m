function [x_temp,y_temp,k_temp,k_deviation_temp]=trajectory_curve_sparse_point(parameter,state_initial)

global number
s_step=abs(parameter(4))/number;
x_temp=zeros(1,number);
y_temp=zeros(1,number);
k_temp=zeros(1,number);
k_deviation_temp=zeros(1,number);
theta=zeros(1,number);
s=zeros(1,number);

s(1)=0;
k_temp(1)=state_initial(4);
k_deviation_temp(1)=parameter(1);
x_temp(1)=state_initial(1);
y_temp(1)=state_initial(2);
theta(1)=state_initial(3);

for i=2:number
    s(i)=s(i-1)+s_step;
    k_temp(i)=state_initial(4)+parameter(1)*s(i)+parameter(2)*s(i)^2+parameter(3)*s(i)^3;
    k_deviation_temp(i)=parameter(1)+2*parameter(2)*s(i)+3*parameter(3)*s(i)^2;
    theta(i)=theta(i-1)+k_temp(i)*s_step;%estimated value
    x_temp(i)=x_temp(i-1)+cos(theta(i))*s_step;%estimated value
    y_temp(i)=y_temp(i-1)+sin(theta(i))*s_step;%estimated value
end

%% Modified by LJX 20161217-10:00
% function [x_temp,y_temp,k_temp,k_deviation_temp]=trajectory_curve_sparse_point(parameter,state_initial)
% 
% %global s_step
% global number
% global rotate_theta
% global plane_phase
% 
% s_step=0.01;
% % flag=0;
% x(1)=state_initial(1);
% y(1)=state_initial(2);
% theta(1)=state_initial(3);
% k_curve(1)=state_initial(4);
% s(1)=0;
% if isnan(parameter(4))
%     steps=100;
% else
%     steps=floor(abs(parameter(4))/s_step);
% end
% % if(steps<number)
% %     flag=1;
% %     return;
% % end
% for i=2:steps
%     s(i)=s(i-1)+s_step;
%     k_curve(i)=k_curve(1)+parameter(1)*s(i)+parameter(2)*s(i)^2+parameter(3)*s(i)^3;
%     k_curve_deviation(i)=parameter(1)+2*parameter(2)*s(i)+3*parameter(3)*s(i)^2;
%     theta(i)=theta(i-1)+k_curve(i)*s_step;
%     x(i)=x(i-1)+cos(theta(i))*s_step;
%     y(i)=y(i-1)+sin(theta(i))*s_step;
% end
% state_terminal=[x(steps), y(steps),theta(steps),k_curve(steps)];
% 
% % plot(s,theta,'g',s,k_curve,'b',s,k_curve_deviation,'r');
% % hold on;
% 
% if rotate_theta~=0
%     if plane_phase==1 %
%         origin(1:length(x),1)=x;
%         origin(1:length(x),2)=y;
%     else
%         origin(1:length(x),1)=x;
%         origin(1:length(x),2)=-y;
%     end
%     shift=[0,0];
%     rotate_theta_temp=-rotate_theta;
%     [x_transform,y_transform]=coordinate_transform(origin,rotate_theta_temp,shift);
%     interval=floor(steps/number);%calculate the final point of the interval
%     if interval>=1
%         for i=1:number
%             x_temp(i)=x(i*interval);
%             y_temp(i)=y(i*interval);
%             k_temp(i)=k_curve(i*interval);
%             k_deviation_temp(i)= k_curve_deviation(i*interval);
%         end
%     else
%         x_temp=x;
%         y_temp=y;
%         k_temp=k_curve;
%         k_deviation_temp=k_curve_deviation;
%     end
% else
%     %  number=floor(steps/10);
%     interval=floor(steps/number);%calculate the final point of the interval
%     if interval>=1
%         for i=1:number
%             x_temp(i)=x(i*interval);
%             y_temp(i)=y(i*interval);
%             k_temp(i)=k_curve(i*interval);
%             k_deviation_temp(i)= k_curve_deviation(i*interval);
%         end
%     else
%         x_temp=x;
%         y_temp=y;
%         k_temp=k_curve;
%         k_deviation_temp=k_curve_deviation;
%     end
%     %     if plane_phase~=1
%     %         y_temp=-y_temp;
%     %     end
% end
% % plot(x_temp,y_temp);
% % grid on;
% % axis([-5 5 -5 5]);
% % axis equal;
% % hold on;
% 
% 
% 
