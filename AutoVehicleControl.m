function [FinalPose]=AutoVehicleControl(velocityprofile,U,XInitial)
%state_calculate(parameter,state_initial)
%% Calculate Final State
global s_step
global iter_t

x(1)=XInitial(1);
y(1)=XInitial(2);
theta(1)=XInitial(3);
k_curve(1)=XInitial(4);
parameter=U;
s(1)=0;
steps=floor(velocityprofile(1)*iter_t/s_step);
for i=2:steps
    s(i)=s(i-1)+s_step;
    k_curve(i)=k_curve(1)+parameter(1)*s(i)+parameter(2)*s(i)^2+parameter(3)*s(i)^3;
    theta(i)=theta(i-1)+k_curve(i)*s_step;
    x(i)=x(i-1)+cos(theta(i))*s_step;
    y(i)=y(i-1)+sin(theta(i))*s_step;
end
if(abs(k_curve(steps))>0.19)%对应前轮摆角约20~30度
    k_curve(steps)=0;
end
% display(theta(steps));
% if(theta(steps)>0.5)
%     theta(steps)=0.5;
% else if (theta(steps)<-0.5)
%     theta(steps)=-0.5;
%     end
% end
FinalPose=[x(steps), y(steps),theta(steps),k_curve(steps)];
end