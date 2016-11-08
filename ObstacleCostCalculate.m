function [distanceObsCan,CollisionFlag,effectivepoint]=ObstacleCostCalculate(ObstacleInforMatrix,CandidatePath,CollisionDistThreshold)

%OBSTACLECOSTCALCULATE Summary of this function goes here
%   Detailed explanation goes here
distanceObsCan=0;
effectivepoint=0;
CollisionFlag=0;

for j=1:size(ObstacleInforMatrix,1)
    distancetemp1=norm(CandidatePath-ObstacleInforMatrix(j,:));
    if(distancetemp1< CollisionDistThreshold)
        CollisionFlag=1;
        break;
    else
        effectivepoint=effectivepoint+1;
        temp=exp(-distancetemp1/10);
        distanceObsCan=distanceObsCan+temp;
    end
end

end




