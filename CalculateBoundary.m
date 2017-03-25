% -------------------------------------------------------------------------
%
% File : CalculateBoundary.m
%
% Discription : Give the boundary to the participant vehicles
% !!This file is only used in PredicitonforTrafficParticipants,though it
% has many similarities to plotroad!
%
% Environment : Matlab
%
% Author : John Lee
%
% Copyright (c): 2016 John Lee
%
% License : Modified BSD Software License Agreement
% -------------------------------------------------------------------------
function [boundary] = CalculateBoundary(environmentFlag)
% Every 10 meters has 100 points
global l_width
l_width=6;
if environmentFlag==1
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
    
    %%%%%%%%%%%%%%%%%% Calculate Boundary-Curve Road %%%%%%%%%%%%%%%%%%%%%%%
    temp1=[30:0.01:firstLineEndX]';
    temp2=-firstLineY*ones(size(temp1,1),1);
    boundaryobstacle=[temp1,temp2];
    temp2=firstLineY*ones(size(temp1,1),1);
    boundaryobstacle=[boundaryobstacle;[temp1,temp2]];
    
    temp1=[firstLineEndX+radius_outer(1)+radius_inner(2):0.01:firstLineEndX+radius_outer(1)+radius_inner(2)+secLineEndX]';
    temp2=(-l_width/2+radius_outer(1)+radius_inner(2))*ones(size(temp1,1),1);
    boundaryobstacle=[boundaryobstacle;[temp1,temp2]];
    temp2=(l_width/2+radius_outer(1)+radius_inner(2))*ones(size(temp1,1),1);
    boundaryobstacle=[boundaryobstacle;[temp1,temp2]];
    
    theta=-pi/2:pi/200:0;
    boundaryobstacle=[boundaryobstacle;[[circlecenter(1,1)+radius_inner(1)*cos(theta)]',[circlecenter(1,2)+radius_inner(1)*sin(theta)]']];
    boundaryobstacle=[boundaryobstacle;[[circlecenter(1,1)+radius_outer(1)*cos(theta)]',[circlecenter(1,2)+radius_outer(1)*sin(theta)]']];
    theta=pi:-pi/200:pi/2;
    boundaryobstacle=[boundaryobstacle;[[circlecenter(2,1)+radius_inner(2)*cos(theta)]',[circlecenter(2,2)+radius_inner(2)*sin(theta)]']];
    boundaryobstacle=[boundaryobstacle;[[circlecenter(2,1)+radius_outer(2)*cos(theta)]',[circlecenter(2,2)+radius_outer(2)*sin(theta)]']];
    
    boundary=boundaryobstacle;
end
end

