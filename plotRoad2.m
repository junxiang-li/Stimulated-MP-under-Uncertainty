function [referencepath_xy] = plotRoad2(figSim)
% This environment depicts only having the straightline in the scene. 
% 
global l_width
l_width=4;

%%%%%%%%%%%%%%%%%% Initiation %%%%%%%%%%%%%%%%%%%%%%%%
firstLineEndX=36;
firstLineY=l_width/2;
% secLineEndX=20;


% radius_outer=[12 12+l_width];
% radius_inner=[radius_outer(1)-l_width 12];
% circlecenter=[firstLineEndX,radius_outer(1)-l_width/2;
%     firstLineEndX+2*radius_outer(1),radius_outer(1)-l_width/2];

straightline=[0,-firstLineY-0.25;
    firstLineEndX,-firstLineY-0.25;
    0,firstLineY+0.25;
    firstLineEndX,firstLineY+0.25;%the first part line %Pay attention:0.5 is for the visual effect
];

%%%%%%%%%%%%%%%%%% Plot the Road%%%%%%%%%%%%%%%%%%%%%%%%
figure(figSim);
for i=1:2:size(straightline)
    plot([straightline(i:i+1,1)],[straightline(i:i+1,2)],'black','LineWidth',2.5);hold on;
end

%%%%%%%%%%%%%%%%%% Calculate Road  Center%%%%%%%%%%%%%%%%%%%%%%%
% pointnum=100;
% x=linspace(0,firstLineEndX,4*pointnum);
% y=linspace(0,0,4*pointnum);
% plot(x,y,'b-.');hold on;

%%%%%%%%%%%%%%%%%% Calculate Reference Road(Road Center)%%%%%%%%%%%%%%%%%%%%%%%
pointnum=100;
x=linspace(0,firstLineEndX,4*pointnum);
y=linspace(0,0,4*pointnum);
theta=linspace(0,0,4*pointnum);
kappa=linspace(0,0,4*pointnum);
temp1=[x;y;theta;kappa];

% x=linspace(firstLineEndX,secLineEndX,pointnum);
% y=linspace(radius_outer(1)+radius_inner(1),radius_outer(1)+radius_inner(1),pointnum);
% theta=linspace(0,0,pointnum);
% kappa=linspace(0,0,pointnum);
% temp5=[x;y;theta;kappa];

referencepath_xy=[temp1];%,temp5
plot(referencepath_xy(1,:),referencepath_xy(2,:),'b-.');hold on;
%axis([5,firstLineEndX,-12,12]);
axis([0,firstLineEndX,-15,15]);%FOR EX1_STRAIGHTLINE
axis off;
%figure(figCur);%@@@@@
%plot(referencepath_xy(1,:),referencepath_xy(4,:),'b');hold on;
end

