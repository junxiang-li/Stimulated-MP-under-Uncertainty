% -------------------------------------------------------------------------
%
% File : Optimization-based Motion Planner With Uncertainty.m
%
% Discription : Mobile Robot Motion Planning with Optimization-based
% Approach considering with prediction and uncertainty
%
% Environment : Matlab
%
% Author : John Lee
%
% Copyright (c): 2016 John Lee
%
% License : Modified BSD Software License Agreement
% -------------------------------------------------------------------------

clc;clear;
global number
global iter_t
global l_width
global v_width
global vehiclelength
global lamda
str='./velocityprofile/';

%load ./Environment/obstacle_in_curve.mat

%%the state of obstacle:[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)],it's a row vector!
%% Experiment1:straight road and curvy road with same effect
obstaclematrix=[14,-1,0,0,0;];
FinalPose=[5,-1,0,0];
lamda=5;
str=strcat(str,'Exp1_1');

% obstaclematrix=[52.5,17,1,0,0;44.2,3.5,0.9,0,0];
% FinalPose=[35,0,0,0];
% lamda=3;
% str=strcat(str,'Exp1_2');
%% Experiment2:a unidirectional two-lane road(two vehicles in front)
% obstaclematrix=[17,-1,0,1,0;14,1,0,4,0;];
% FinalPose=[10,1,0,0];
%lamda=7;%without prediction
%str=strcat(str,'Exp2');
% lamda=5;%with prediction
%% Experiment3:a unidirectional two-lane road(two vehicles, one is overtaking ego vehilce)
%obstaclematrix=[8.5,-1,0,5.5,0;15.5,1,0,2,0;];
%FinalPose=[10,1,0,0];
%lamda=7;%change w in the costfunction!!!!!!!
%str=strcat(str,'Exp3');
%% Experiment4:a bidirectional two-lane road(two vehicles with opposite direction)
% obstaclematrix=[15,-1,0,8,0;27,1,0,-4,0;];
% FinalPose=[10,1,0,0];
% lamda=3;
% str=strcat(str,'Exp4');


obstacle=obstaclematrix;
load full_tree_onelayer.mat
figSim=figure('NumberTitle','off','Name','Simulation');
%figCur=figure('NumberTitle','off','Name','Curvature');
%figVel=figure('NumberTitle','off','Name','Velocity');
figure(figSim);
%%%%%%%%%%%%%%%%% Display the Environment %%%%%%%%%%%%%%%%
referencepath_xy=plotRoad2(figSim);%straightline
%referencepath_xy=plotRoad3(figSim);%curveline

%%%%%%%Function JudgeFinalCenterPoint Parameter%%%%%%%%%%%%%%%%
endflag=0;%0 means normal,1 means stop!
d_horizon=40; %horizon index(50 points ahead, not the length)

%%%%%%%Function GenerateUniformBoundaryStates Parameter%%%%%%%%%%%%%%%%
l_heading=0;%the parameter could be tested by the perception system
l_width=4;% the width of the lane
v_width=1; % the width of the vehicle
vehiclelength=2;% the length of the vehicle
d=10;% horizon forward
n_p=15;% the number of lateral offsets to generate for each lane
n_l=1;% the number of lanes itself

%%%%%%Function Parametric_Trajectory_Generation Parameter%%%%%%%%%%%%%%%%
parameter_previous=[0,0,0,0]';
LookUpTable=lookup_table;

%%%%%%Function ComputeTrajectoryCost() Parameter%%%%%%%%%%%%%%%%
number=50;
Cost=1;
times=1;

%%%%%%Car Control Parameter%%%%%%%%%%%%%%%%
iter_t=0.5;%planning cycle:100ms
velocitycurve=zeros(number+1,4);

%%%%%%%Section Prediction Parameter%%%%%%%%%%%%%%%%
predictflag=0;%predictflag==0 means it has no dynamic obstacle prediction,predictflag==1 means it has prediction
predictobstacle=obstaclematrix;%the predict state of obstacle:[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
predictCovarianceMatrixSet=zeros(5,5,size(obstaclematrix,1));%only has covariance of position[x,y]
if predictflag==1
    for pn=1:size(predictCovarianceMatrixSet,3)
        predictCovarianceMatrixSet(:,:,pn)=eye(5);
    end
    QCovariance(:,:,1)=mdiag([0.0438,0.0141;0.0255,0.0266],0,eye(2));
    QCovariance(:,:,2)=mdiag(0.5*diag([abs(randn(1)),abs(randn(1))]),0,eye(2));
    str=strcat(str,'p');
end
%trajectory_curve_draw(U(:,finalparameterindex),XInitial);
%% Motion Planning Algorithm %%%%%%%%%%%%%%%%
for PlanIter=1:50
    %%  Plot Vehicle Rectangle%%%%%%%%%%%%%%%%%%%%%%%%%
    plotvehiclerectangle([FinalPose(1),FinalPose(2)],FinalPose(3),v_width,vehiclelength,[1,0.25,0.25]);%[96 96 96]/255
    %%  Plot Obstacles %%%%%%%%%%%%%%%%
    figure(figSim);
    obstaclenum=size(obstacle,1);
    tempobstacle=obstacle;k=0;obstacle=[];
    for i=1:obstaclenum
        if (tempobstacle(i,1)>=min(referencepath_xy(1,:))&&tempobstacle(i,1)<=max(referencepath_xy(1,:))&&tempobstacle(i,2)>=(min(referencepath_xy(2,:))-l_width)&&tempobstacle(i,2)<=(max(referencepath_xy(2,:))+l_width))
            plotvehiclerectangle([tempobstacle(i,1),tempobstacle(i,2)],tempobstacle(i,3),v_width,vehiclelength,[0.8,0.8,0.8]);hold on;%straightline%yellow:[1,0.9,0]%gray:[0.8,0.8,0.8]%[1,0.9,0]
            k=k+1;
            obstacle(k,:)=tempobstacle(i,:);
        end
    end
    %% Motion Planning Initialization-Sample
    MinCost=1;
    parameter_inherit_flag=0;
    XInitial=FinalPose;%[x,y,\theta,\kappa]
    XInitialPosition=[XInitial(1),XInitial(2)]';
    [FinalPoint,endflag]=JudgeFinalCenterPoint(referencepath_xy,XInitial',d_horizon);
    XF=GenerateUniformBoundaryStates(FinalPoint,l_width,v_width,n_p,n_l,XInitial);
    referencepath=TrimReferencePath(referencepath_xy,XInitialPosition,FinalPoint);
    if endflag==1
        %curvature_draw(U(:,finalparameterindex),XInitial,figSim);%This function combines the trajectory_curve_draw(U(:,finalparameterindex),XInitial) and plot the vehicle.
        break;
    end
    SamplePointNum=size(XF,2);
    U=zeros(4,SamplePointNum);
    %% Prediction
    if predictflag==1
        for j=1:size(obstacle,1)
            initialobstaclestate=obstacle(j,:);
            if initialobstaclestate(4)>0
                GoalState=referencepath_xy(1:2,end);
            elseif initialobstaclestate(4)<0
                GoalState=XInitialPosition;%(when at circumstance of interaction)
            elseif initialobstaclestate(4)==0
                GoalState=[initialobstaclestate(1),initialobstaclestate(2)];
            end
            OtherObstacle=obstaclematrix;%the obstaclematrix will upgraded in each iteration
            OtherObstacle(j,:)=[];
            P_init=predictCovarianceMatrixSet(:,:,j);
            Q_init=QCovariance(:,:,j);
            [PredictState,PredictVariance]=PredictionforTrafficParticipants(initialobstaclestate,GoalState,OtherObstacle,P_init,Q_init);
            predictobstacle(j,:)=PredictState;
            predictCovarianceMatrixSet(:,:,j)=PredictVariance;
        end
    end
    %% Generation-Evaluation
    for i=1:SamplePointNum
        parameter_previous=ParameterInitiation(XInitial,XF(:,i)',LookUpTable);
        [parameter,final_state]=Parametric_Trajectory_Generation(parameter_previous',XInitial,XF(:,i)',parameter_inherit_flag);
        %trajectory_curve_draw(parameter,XInitial);
        [velocityprofile]=VelocityPlanning(XInitial,parameter(4));
        U(:,i)=parameter';
        S(:,i)=velocityprofile';
        if  any(U(:,i)) ~= 0 & U(:,i)~= Inf & parameter(4)>0.2 & parameter(4)<10
            %disp(i);
            Cost=ComputeTrajectoryCost(XInitial,U(:,i),velocityprofile,referencepath',obstacle,predictflag,predictobstacle,predictCovarianceMatrixSet);
        end
        if Cost<MinCost
            finalparameterindex=i;
            MinCost=Cost;
        end
    end
    if MinCost>1
        finalparameterindex=floor(SamplePointNum/2);
    end
    
    %Plot the result
    curvature_draw(U(:,finalparameterindex),XInitial,figSim);%This function combines the trajectory_curve_draw(U(:,finalparameterindex),XInitial) and plot the vehicle.
    %% Perceptive and Localization Information
    NewDynamicObstacleState=DynamicControl(obstacle);
    obstacle=NewDynamicObstacleState;
    
    [FinalPose]=AutoVehicleControl(S(:,finalparameterindex),U(:,finalparameterindex),XInitial);%FinalPose=XF(:,finalparameterindex)';
    velocitycurve(:,PlanIter)=[PlanIter;S(:,finalparameterindex)];
end
save(str,'velocitycurve');





