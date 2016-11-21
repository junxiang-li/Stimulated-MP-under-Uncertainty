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
global l_width
load full_tree_onelayer.mat
load ./Environment/obstacle_in_curve.mat
%load ./Environment/obstacle_in_straight.mat
figSim=figure('NumberTitle','off','Name','Simulation');
figCur=figure('NumberTitle','off','Name','Curvature');
figure(figSim);

%%%%%%%Function GenerateUniformBoundaryStates() Parameter%%%%%%%%%%%%%%%%
l_heading=0;%the parameter could be tested by the perception system
l_width=6;% the width of the lane
v_width=1; % the width of the vehicle
d=10;% horizon forward
n_p=15;% the number of lateral offsets to generate for each lane
n_l=1;% the number of lanes itself
d_horizon=50;

%%%%%%Function Parametric_Trajectory_Generation() Parameter%%%%%%%%%%%%%%%%
parameter_previous=[0,0,0,0]';
LookUpTable=lookup_table;

%%%%%%Function ComputeTrajectoryCost() Parameter%%%%%%%%%%%%%%%%
number=50;
Cost=1;
FinalPose=[30,0,0,0];
endflag=0;%0 means normal,1 means stop!
times=1;

%%%%%%%%%%% Plot Obstacles %%%%%%%%%%%%%%%%
obstaclematrix=obstaclematrix;
% n=length(obstaclematrix);
% for i=1:n
     %plotvehiclerectangle([obstacle(i,1),obstacle(i,2)],0,0.5,1,'y');hold on;%straightline
     %plotvehiclerectangle([obstacle(i,1),obstacle(i,2)],1,0.5,1,'y');hold on;%curve
% end
plotvehiclerectangle([49.3,5],1,0.5,1,'y');hold on;
plotvehiclerectangle([52.5,17],0.8,0.5,1,'y');

%%%%%%%%%%%%%%%%% Display the Environment %%%%%%%%%%%%%%%%
%referencepath_xy=plotRoad2(figSim);%straightline
referencepath_xy=plotRoad3(figSim);%curveline


%trajectory_curve_draw(U(:,finalparameterindex),XInitial);
figure(figSim);
%%%%%%%%%%%%%%%%% Motion Planning Algorithm %%%%%%%%%%%%%%%%
while(1)
    MinCost=1;
    parameter_inherit_flag=0;
    XInitial=FinalPose;%[x,y,\theta,\kappa]
    XInitialPosition=[XInitial(1),XInitial(2)]';
    [FinalPoint,endflag]=JudgeFinalCenterPoint(referencepath_xy,XInitial',d_horizon);
    XF=GenerateUniformBoundaryStates(FinalPoint,l_width,v_width,n_p,n_l,XInitial);
    referencepath=TrimReferencePath(referencepath_xy,XInitialPosition,FinalPoint);
    if endflag==1
        break;
    end
    SamplePointNum=size(XF,2);
    U=zeros(4,SamplePointNum);
    
    for i=1:SamplePointNum
        parameter_previous=ParameterInitiation(XInitial,XF(:,i)',LookUpTable);
        [parameter,final_state]=Parametric_Trajectory_Generation(parameter_previous',XInitial,XF(:,i)',parameter_inherit_flag);
        %trajectory_curve_draw(parameter,XInitial);
        U(:,i)=parameter';
        if  any(U(:,i)) ~= 0 & U(:,i)~= Inf
            Cost=ComputeTrajectoryCost(XInitial,U(:,i),referencepath',obstaclematrix);
        end
        if Cost<=MinCost
            finalparameterindex=i;
            MinCost=Cost;
        end
    end
    if MinCost==1
        finalparameterindex=floor(SamplePointNum/2);
    end
    FinalPose=XF(:,finalparameterindex)';
    %Plot the result
    curvature_draw(U(:,finalparameterindex),XInitial,v_width/2,figSim,figCur);%This function combines the trajectory_curve_draw(U(:,finalparameterindex),XInitial) and plot the vehicle.
    times=times+1;
end

 FigureImprovement(figSim);
 FigureImprovement(figCur);





