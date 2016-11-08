function [referencepath_xy] = plotRoad3(figSim)
% Every 10 meters has 100 points
% ,figCur%@@@@@%
global l_width
l_width=6;

%%%%%%%%%%%%%%%%%% Initiation %%%%%%%%%%%%%%%%%%%%%%%%

firstLineEndX=40;
firstLineY=l_width/2;
secLineEndX=20;


radius_outer=[12 12+l_width];
radius_inner=[radius_outer(1)-l_width 12];
circlecenter=[firstLineEndX,radius_outer(1)-l_width/2;
    firstLineEndX+2*radius_outer(1),radius_outer(1)-l_width/2];

straightline=[30,-firstLineY;
    firstLineEndX,-firstLineY;
    30,firstLineY;
    firstLineEndX,firstLineY;%the first part line
    
    firstLineEndX+radius_outer(1)+radius_inner(2),-l_width/2+radius_outer(1)+radius_inner(2);
    firstLineEndX+radius_outer(1)+radius_inner(2)+secLineEndX,-l_width/2+radius_outer(1)+radius_inner(2);
    firstLineEndX+radius_outer(1)+radius_inner(2),l_width/2+radius_outer(1)+radius_inner(2);
    firstLineEndX+radius_outer(1)+radius_inner(2)+secLineEndX,l_width/2+radius_outer(1)+radius_inner(2);
];

%%%%%%%%%%%%%%%%%% Plot the Road%%%%%%%%%%%%%%%%%%%%%%%%
figure(figSim);
for i=1:2:size(straightline)
    plot([straightline(i:i+1,1)],[straightline(i:i+1,2)],'black','LineWidth',2.5);hold on;
end

theta=-pi/2:pi/100:0;
plot(circlecenter(1,1)+radius_inner(1)*cos(theta),circlecenter(1,2)+radius_inner(1)*sin(theta),'black','LineWidth',2.5);
hold on;
plot(circlecenter(1,1)+radius_outer(1)*cos(theta),circlecenter(1,2)+radius_outer(1)*sin(theta),'black','LineWidth',2.5);
axis equal;

theta=pi:-pi/100:pi/2;
plot(circlecenter(2,1)+radius_inner(2)*cos(theta),circlecenter(2,2)+radius_inner(2)*sin(theta),'black','LineWidth',2.5);
hold on;
plot(circlecenter(2,1)+radius_outer(2)*cos(theta),circlecenter(2,2)+radius_outer(2)*sin(theta),'black','LineWidth',2.5);

%%%%%%%%%%%%%%%%%% Calculate Reference Road %%%%%%%%%%%%%%%%%%%%%%%
pointnum=100;
x=linspace(30,firstLineEndX,pointnum);
y=linspace(0,0,pointnum);
theta=linspace(0,0,pointnum);
kappa=linspace(0,0,pointnum);
temp1=[x;y;theta;kappa];

theta=-pi/2:pi/300:0;%-pi/2:pi/200:0;
x=circlecenter(1,1)+(radius_inner(1)+radius_outer(1))/2*cos(theta);
y=circlecenter(1,2)+(radius_inner(1)+radius_outer(1))/2*sin(theta);
theta1=0:pi/300:pi/2;%set the reference road a global coordinate
kappa1=linspace(1/(radius_inner(1)+radius_outer(1)),1/(radius_inner(1)+radius_outer(1)),size(theta1,2));
temp2=[x;y;theta1;kappa1];

%
theta=pi:-pi/500:pi/2;%pi:-pi/200:pi/2;
x2=circlecenter(2,1)+(radius_inner(2)+radius_outer(2))/2*cos(theta);
y2=circlecenter(2,2)+(radius_inner(2)+radius_outer(2))/2*sin(theta);
theta2=pi/2:-pi/500:0;
kappa2=linspace(1/(radius_inner(1)+radius_outer(1)),1/(radius_inner(1)+radius_outer(1)),size(theta2,2));
temp3=[x2;y2;theta2;kappa2];

x=linspace(firstLineEndX+radius_outer(1)+radius_inner(2),firstLineEndX+radius_outer(1)+radius_inner(2)+secLineEndX,2*pointnum);
y=linspace(radius_outer(1)+radius_inner(2),radius_outer(1)+radius_inner(2),2*pointnum);
theta=linspace(0,0,2*pointnum);
kappa=linspace(0,0,2*pointnum);
temp4=[x;y;theta;kappa];

% x=linspace(firstLineEndX,secLineEndX,pointnum);
% y=linspace(radius_outer(1)+radius_inner(1),radius_outer(1)+radius_inner(1),pointnum);
% theta=linspace(0,0,pointnum);
% kappa=linspace(0,0,pointnum);
% temp5=[x;y;theta;kappa];

referencepath_xy=[temp1,temp2,temp3,temp4];%,temp5
plot(referencepath_xy(1,:),referencepath_xy(2,:),'b-.');
axis equal;
%figure(figCur);%@@@@@
%plot(referencepath_xy(1,:),referencepath_xy(4,:),'b');hold on;
end

