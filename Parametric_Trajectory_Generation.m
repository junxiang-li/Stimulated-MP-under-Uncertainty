
function [parameter,state_terminal1]=Parametric_Trajectory_Generation(parameter_previous,state_initial,state_terminal,parameter_inherit_flag)

% parmeter_previous=[0,0,0,5];
% state_initial=[0,0,0,0];
% state_terminal=[5,0,-0.1745,0]
% parameter_inherit_flag=0;
%%%%%%���ó�ʼ״̬%%%%%%%
%%%%%%���賵��״̬Ϊ4D״̬�ռ�(x ,y ,theta ,k_curve),(x,y)Ϊλ�ã�thetaΪ����ǣ�k_curveΪ����
global rotate_theta  %����ȫ�ֱ���_����ʼ����ǲ�Ϊ��ʱ����ô�����Ҫ��ת�ĽǶȾ�Ϊrotate_theta
% rotate_theta=state_initial(3);
% if  rotate_theta~=0
%     state_initial=[0 0 0 0];
%     x_temp=state_terminal(1);
%     y_temp=state_terminal(2);
%     state_terminal(1)=x_temp*cos(rotate_theta)+y_temp*sin(rotate_theta);%��ʼ����ǲ�Ϊ��ʱ����Ҫ���Ƚ�����ת
%     state_terminal(2)=-x_temp*sin(rotate_theta)+y_temp*cos(rotate_theta);
%     state_terminal(3)=state_terminal(3)-rotate_theta;
% end

k_curve(1)=state_initial(4);
yacobian_matrix=zeros(4,4);%��ʼ��yacobian����4*4�ķ���
parameter_adj=[1e-6, 1e-6,1e-6,1e-3*5]; % �������yacobian����ʱ�Ĳ���仯������
state_error_weight=[50 50 50 50]; % ����״̬����Ȩ�أ��ж��Ƿ�������������
max_iteration=6;   %���������Ĵ�������˵�����û�������Ļ�����ô����Ϊ���޽⡣

%%%%%%%%�Բ�����г�ʼ��%%%%%%%%%%%%%%
x_terminal=state_terminal(1);
y_terminal=state_terminal(2);
theta_terminal=state_terminal(3);
k_curve_terminal=state_terminal(4);

%     d=sqrt(x_terminal^2+y_terminal^2);
%     s_initial=d*(theta_terminal^2/5+1)+2*abs(theta_terminal)/5;
%     a_initial=6*theta_terminal/(s_initial^2)-2*k_curve(1)/s_initial+4*k_curve_terminal/s_initial;
%     b_initial=3*(k_curve(1)+k_curve_terminal)/(s_initial^2)+6*theta_terminal/(s_initial^3);
%     c_initial=0;
%     parameter_initial=[a_initial b_initial c_initial s_initial];%��ʼ������
%    parameter=parameter_initial;
parameter=parameter_previous;

for k=1:max_iteration
    state_terminal1=state_calculate(parameter,state_initial);
    state_error=state_terminal-state_terminal1;%����ĩ��״̬���
    
    index=k;
    state_error_1=abs(state_error_weight(1)*state_error(1));
    state_error_2=abs(state_error_weight(2)*state_error(2));
    state_error_3=abs(state_error_weight(3)*state_error(3));
    state_error_4=0*abs(state_error_weight(4)*state_error(4));
    if (state_error_1)<1&&(state_error_2<1)&&(state_error_3<1)&&(state_error_4<1)%%���������ֵ
        %state_error=state_terminal-state_terminal_1 ; %%%��ʼ����õ�״̬���
        %trajectory_curve_draw(parameter,state_initial); %���û����ߵĺ���
        break; %�����һ��forѭ��
    end
    state_terminal_current = state_terminal1;
    delt_state_teminal_current = state_terminal-state_terminal_current;
    
    %%%%%%%%%%%%%%%��jacobian����%%%%%%%%%%%%%
    for i=1:length(parameter)
        %%%%%%%%%%%����һ
        parameter_1=parameter;
        parameter_1(i)=parameter(i)+parameter_adj(i);   %��ÿ���������΢����ͨ����������������ȷ�ı仯���ơ�
        %[Q,state_terminal1]=FirstDerivatives(parameter_1,state_initial);
        state_terminal1=state_calculate(parameter_1,state_initial);
        state_error=state_terminal-state_terminal1;
        current = (state_error - delt_state_teminal_current)./parameter_adj(i);%������ֵ�������ƫ��
        yacobian_matrix(1:4,i)=current;
    end
    state_error=delt_state_teminal_current;  %�۲������У�״̬���Ľ��
    delta_parameter=-inv(yacobian_matrix)*state_error';  %%�����仯��С
    parameter=delta_parameter'+parameter; %�õ��µĲ���
    Q(:,k)=parameter;
end







