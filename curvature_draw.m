function curvature_draw(parameter,state_initial,figSim)

global s_step
global vehiclelength
global v_width

x(1)=state_initial(1);
y(1)=state_initial(2);
theta(1)=state_initial(3);
k_curve(1)=state_initial(4);
s(1)=0;
steps=floor(abs(parameter(4))/s_step);%steps=floor(parameter(4)/s_step);
for i=2:steps
    s(i)=s(i-1)+s_step;
    k_curve(i)=k_curve(1)+parameter(1)*s(i)+parameter(2)*s(i)^2+parameter(3)*s(i)^3;
    theta(i)=theta(i-1)+k_curve(i)*s_step;
    x(i)=x(i-1)+cos(theta(i))*s_step;
    y(i)=y(i-1)+sin(theta(i))*s_step;
end
state_terminal=[x(steps), y(steps),theta(steps),k_curve(steps)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Plot Planning Result%%%%%%%%%%%%%%%%%%%%%%%%%
number=floor(steps/10);
for i=1:number
    x_temp(i)=x(i*10);
    y_temp(i)=y(i*10);
    k_temp(i)=k_curve(i*10);
end
figure(figSim);
plot(x_temp,y_temp,'color',[1,0.25,0.25],'Linewidth',1.5);hold on;






