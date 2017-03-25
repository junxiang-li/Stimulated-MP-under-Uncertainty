% -------------------------------------------------------------------------
%
% File : PredictionforTrafficParticipants.m
%
% Discription : Prediction of Vehicle with Dynamic Window Approach and
% Kalman Filter
%
% Environment : Matlab
%
% Author : John Lee
%
% Copyright (c): 2016 John Lee
%
% License : Modified BSD Software License Agreement
% -------------------------------------------------------------------------

function [PredictState,PredictVariance] = PredictionforTrafficParticipants(startpoint,destpoint,OtherObstacle,P_init,Q_init)

x=startpoint';%x=[30,0,0,3,0]';% 机器人的初期状态[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
goal=destpoint;%goal=[70,22];% 目标点位置 [x(m),y(m)]
obstacle=OtherObstacle(:,1:2);
n=length(obstacle);
%P_update(:,:,1)=ones(5);
P_update(:,:,1)=P_init;

%%  Road Boundary Obstacles %%%%%%%%%%%%%%%%
%environmentFlag=0;%0:line,1:curveline
%boundary_xy=CalculateBoundary(environmentFlag);%curveline
%obstacle=[obstacle(1:2,:);boundary_xy];

%% 机器人运动学模型
% 最高速度[m/s],最高旋转速度[rad/s],加速度[m/ss],旋转加速度[rad/ss],
% 速度分辨率[m/s],转速分辨率[rad/s]
Kinematic=[10.0,toRadian(30.0),1,toRadian(20.0),0.05,toRadian(1)];
obstacleR = 0.5;% 冲突判定用的障碍物半径
global dt; dt=0.1;% 时间[s]
A = [1 0 0 0 0
    0 1 0 0 0
    0 0 1 0 0
    0 0 0 0 0
    0 0 0 0 0];
B = [dt*cos(x(3)) 0
    dt*sin(x(3)) 0
    0 dt
    1 0
    0 1];
C =[1 0 0 0 0
    0 1 0 0 0];
x_predict(:,1)=x;
H=[1 0 0 0 0
    0 1 0 0 0];
Q=Q_init;
% Q = mdiag([0.5,0;0,0.4],0,eye(2));%mdiag(0.5*diag([abs(randn(1)),abs(randn(1))]),0,eye(2));
% R = [0.5,0;0,0.02];%diag([0.3*rand(1),0.03*rand(1)]);
%Q=mdiag([0.0438,0.0141;0.0255,0.0266],0,eye(2));
R=[0.05,0;0,0.02];;%diag([0.3*rand(1),0.03*rand(1)]);
mu = [0 0]; Sigma = R;

%% 评价函数参数 [heading,dist,velocity,predictDT]
evalParam=[0.05,0.2,0.1,0.5];%predictDT=3.0
%area=[30 60 -15 15];% 模拟区域范围 [xmin xmax ymin ymax]

%% DWA Approach
%disp('Dynamic Window Approach sample program start!!')
i=2;
[u,traj]=DynamicWindowApproach(x,Kinematic,goal,evalParam,obstacle,obstacleR);
% disp('u');disp(u);
% disp('traj');disp(traj);
%% Use Kalman Filter to predict the state of robotic%%%%%%%%%%%%%%%%%%%%
% 误差为服从正太分布的随机变量
r = mvnrnd(mu, Sigma, 1)';
z(:,i)=C*x+r;%normrnd(0,1); %ObservationEquation
% 误差为一个较小的随机值，其性能表现也仍然与P，Q值的选择有较大的影响！
% z(:,i)=C*x+0.01*normrnd(0,1); %ObservationEquation
%-----1. 预测-----
%-----1.1 预测状态-----
u_record(:,i)=u;
x_predict(:,i)=f(x,u);% 机器人移动到下一个时刻
%-----1.2 预测误差协方差-----
P_predict(:,:,i)=A*P_update(:,:,i-1)*A'+Q;%p1为一步估计的协方差，此式从t-1时刻最优化估计s的协方差得到t-1时刻到t时刻一步估计的协方差

%-----2. 更新-----
%-----2.1 计算卡尔曼增益-----
K(:,:,i)=P_predict(:,:,i)*H' / (H*P_predict(:,:,i)*H'+R);%K(t)为卡尔曼增益，其意义表示为测量与预测的权重比
%-----2.2 更新状态-----
x_update(:,i)=x_predict(:,i)  +  K(:,:,i) * (z(:,i)-H*x_predict(:,i));%Y(t)-a*c*s(t-1)称之为新息，是观测值与一步估计得到的观测值之差，此式由上一时刻状态的最优化估计s(t-1)得到当前时刻的最优化估计s(t)
%-----2.3 更新误差协方差-----
P_update(:,:,i)=P_predict(:,:,i) - K(:,:,i)*H*P_predict(:,:,i);%此式由一步估计的协方差得到此时刻最优化估计的协方差

%% 处理与输出
x=x_update(:,i);
P=inv(P_update(:,:,i));
% if norm(x(1:2)-goal')<0.5
%     disp('Watch out for collision!!');break;
% end
% 预计行驶轨迹
ArrowLength=3;
%quiver(x(1),x(2),ArrowLength*cos(x(3)),ArrowLength*sin(x(3)),'ok');hold on; %'ok' sets the line without arrow,the start point set to be 'o'
%探索轨迹
% if ~isempty(traj)
%     for it=1:length(traj(:,1))/5
%         ind=1+(it-1)*5;
%         plot(traj(ind,:),traj(ind+1,:),'*g');hold on;
%     end
% end

% 预计分布范围 [绘制协方差矩阵（高斯随机值）]
uncertain_u=[x(1) x(2)]';
uncertain_P=P([1,2],[1,2]);
r=chi2inv(0.95,2);
ellipsefig(uncertain_u,uncertain_P,r,1);% 画一般椭圆或椭球：(x-xc)'*P*(x-xc) = r
drawnow;
% 输出结果
PredictState=x;
PredictVariance=P;
%disp('PredictVariance');disp(PredictVariance);

function [u,trajDB]=DynamicWindowApproach(x,model,goal,evalParam,ob,R)

% Dynamic Window [vmin,vmax,wmin,wmax]
Vr=CalcDynamicWindow(x,model);

% 评价函数的计算
[evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam);

if isempty(evalDB)
    u=[0;0];return;
end

% 各评价函数正则化
evalDB=NormalizeEval(evalDB);

% 最终评价函数的计算
feval=[];
for id=1:length(evalDB(:,1))
    feval=[feval;evalParam(1:3)*evalDB(id,3:5)'];
end
evalDB=[evalDB feval];

[maxv,ind]=max(feval);% 最优评价函数
u=evalDB(ind,1:2)';%

function [evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam)
%
evalDB=[];
trajDB=[];
for vt=Vr(1):model(5):Vr(2)
    for ot=Vr(3):model(6):Vr(4)
        % 轨迹推测; 得到 xt: 机器人向前运动后的预测位姿; traj: 当前时刻 到 预测时刻之间的轨迹
        [xt,traj]=GenerateTrajectory(x,vt,ot,evalParam(4),model);  %evalParam(4),前向模拟时间;
        % 各评价函数的计算
        heading=CalcHeadingEval(xt,goal);
        dist=CalcDistEval(xt,ob,R);
        vel=abs(vt);
        % 制动距离的计算
        stopDist=CalcBreakingDist(vel,model);
        evalDB=[evalDB;[vt ot heading dist vel]];
        trajDB=[trajDB;traj];
%         if dist>stopDist %
%         end
    end
end

function EvalDB=NormalizeEval(EvalDB)
% 评价函数正则化
if sum(EvalDB(:,3))~=0
    EvalDB(:,3)=EvalDB(:,3)/sum(EvalDB(:,3));
end
if sum(EvalDB(:,4))~=0
    EvalDB(:,4)=EvalDB(:,4)/sum(EvalDB(:,4));
end
if sum(EvalDB(:,5))~=0
    EvalDB(:,5)=EvalDB(:,5)/sum(EvalDB(:,5));
end

function [x,traj]=GenerateTrajectory(x,vt,ot,evaldt,model)
% 轨迹生成函数
% evaldt：前向模拟时间; vt、ot当前速度和角速度;
global dt;
time=0;
u=[vt;ot];% 输入值
traj=x;% 机器人轨迹
while time<=evaldt
    time=time+dt;% 时间更新
    x=f(x,u);% 运动更新
    traj=[traj x];
end

function stopDist=CalcBreakingDist(vel,model)
% 根据运动学模型计算制动距离,这个制动距离并没有考虑旋转速度，不精确吧！！！
global dt;
stopDist=0;
while vel>0
    stopDist=stopDist+vel*dt;% 制动距离的计算
    vel=vel-model(3)*dt;%
end

function dist=CalcDistEval(x,ob,R)
% 障碍物距离评价函数
dist=100;
for io=1:length(ob(:,1))
    disttmp=norm(ob(io,:)-x(1:2)')-R;
    if dist>disttmp% 离障碍物最小的距离
        dist=disttmp;
    end
end
% 障碍物距离评价限定一个最大值，如果不设定，一旦一条轨迹没有障碍物，将太占比重
% if dist>=2*R
%     dist=2*R;
% end


function heading=CalcHeadingEval(x,goal)
% heading的评价函数计算

theta=toDegree(x(3));% 机器人朝向
goalTheta=toDegree(atan2(goal(2)-x(2),goal(1)-x(1)));% 目标点的方位

if goalTheta>theta
    targetTheta=goalTheta-theta;% [deg]
else
    targetTheta=theta-goalTheta;% [deg]
end

heading=180-targetTheta;

function Vr=CalcDynamicWindow(x,model)
%
global dt;
% 车子速度的最大最小范围
Vs=[0 model(1) -model(2) model(2)];

% 根据当前速度以及加速度限制计算的动态窗口
Vd=[x(4)-model(3)*dt x(4)+model(3)*dt x(5)-model(4)*dt x(5)+model(4)*dt];

% 最终的Dynamic Window
Vtmp=[Vs;Vd];
Vr=[max(Vtmp(:,1)) min(Vtmp(:,2)) max(Vtmp(:,3)) min(Vtmp(:,4))];

function x = f(x, u)
% Motion Model
% u = [vt; ot];当前时刻采样的速度、角速度
global dt;
A = [1 0 0 0 0
    0 1 0 0 0
    0 0 1 0 0
    0 0 0 0 0
    0 0 0 0 0];

B = [dt*cos(x(3)) 0
    dt*sin(x(3)) 0
    0 dt
    1 0
    0 1];

x= A*x+B*u;

function radian = toRadian(degree)
% degree to radian
radian = degree/180*pi;

function degree = toDegree(radian)
% radian to degree
degree = radian/pi*180;



