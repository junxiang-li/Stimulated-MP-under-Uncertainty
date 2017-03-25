function [state_terminal]=state_calculate(parameter,state_initial)
%parameter=[   -1.0181    0.0318   -0.0001 -197.1080];
%state_initial=[   40.0000   -0.3571         0         0];

global s_step; % ����ȫ�ֱ���
s_step=0.001;

x(1)=state_initial(1);
y(1)=state_initial(2);
theta(1)=state_initial(3);
k_curve(1)=state_initial(4);
s(1)=0;
if isnan(parameter(4))
    steps=100;
else
    steps=floor(abs(parameter(4))/s_step);
end

%%%ͨ���ֵķ�ʽ���ĩ��״̬��?
if steps==0
    state_terminal=[x,y,theta,k_curve];
else
    for i=2:steps
        s(i)=s(i-1)+s_step;
        k_curve(i)=k_curve(1)+parameter(1)*s(i)+parameter(2)*s(i)^2+parameter(3)*s(i)^3;
        theta(i)=theta(i-1)+k_curve(i)*s_step;
        x(i)=x(i-1)+cos(theta(i))*s_step;
        y(i)=y(i-1)+sin(theta(i))*s_step;
    end
    state_terminal=[x(steps), y(steps),theta(steps),k_curve(steps)];
end
% plot(s,k_curve);
% hold on;
% plot(x,y,'*');
% hold on;