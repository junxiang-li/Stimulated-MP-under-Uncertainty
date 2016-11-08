function trajectory_curve_draw(parameter,state_initial)

global s_step
global rotate_theta
global plane_phase %
rotate_theta=0;
x(1)=state_initial(1);
y(1)=state_initial(2);
theta(1)=state_initial(3);
k_curve(1)=state_initial(4);
s(1)=0;
steps=floor(parameter(4)/s_step);
for i=2:steps
    s(i)=s(i-1)+s_step;
    k_curve(i)=k_curve(1)+parameter(1)*s(i)+parameter(2)*s(i)^2+parameter(3)*s(i)^3;
    theta(i)=theta(i-1)+k_curve(i)*s_step;
    x(i)=x(i-1)+cos(theta(i))*s_step;
    y(i)=y(i-1)+sin(theta(i))*s_step;
end
state_terminal=[x(steps), y(steps),theta(steps),k_curve(steps)];
%plot(s,k_curve);
%hold on;

if rotate_theta~=0
    if plane_phase==1
        origin(1:length(x),1)=x;
        origin(1:length(x),2)=y;
    else
        origin(1:length(x),1)=x;
        origin(1:length(x),2)=-y;
    end
    shift=[0,0];
    rotate_theta_temp=-rotate_theta;
    [x_transform,y_transform]=coordinate_transform(origin,rotate_theta_temp,shift);
    number=floor(steps/10);
    %%%%%%���õ���·������ϡ�軯��ʾ%%%%%%
    for i=1:number
        x_temp(i)=x_transform(i*10);
        y_temp(i)=y_transform(i*10);
    end
    plot(x_temp,y_temp,'r');
    hold on;
else
    %%%%%%���õ���·������ϡ�軯��ʾ%%%%%%
    number=floor(steps/10);
    for i=1:number
        x_temp(i)=x(i*10);
        y_temp(i)=y(i*10);
    end
    plot(x_temp,y_temp,'r');
    hold on;
end
grid on;
hold on;



