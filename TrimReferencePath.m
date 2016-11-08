function [ LocalReferencePath ] = TrimReferencePath( GlobalReferencePath,InitialPosition,FinalPosition )
%TRIMREFERENCEPATH Summary of this function goes here
%   Detailed explanation goes here
%
%temppoints=IntricatePoints(InitialPositon,FinalPositon,GlobalReferencePath);
endflag=0;
MinExecuablePoints=10;
for i=1:size(GlobalReferencePath,2)
    InitialNormMatrix(:,i)=norm(GlobalReferencePath(1:2,i)-InitialPosition(1:2));
    FinalNormMatrix(:,i)=norm(GlobalReferencePath(1:2,i)-FinalPosition(1:2));
end
[Initial_x,Initial_y]=find(InitialNormMatrix==min(InitialNormMatrix),1);
[Final_x,Final_y]=find(FinalNormMatrix==min(FinalNormMatrix),1);
LocalReferencePath=GlobalReferencePath(:,Initial_y:Final_y);
if size(LocalReferencePath,2)<MinExecuablePoints
    endflag=1;
end
end

