function [Q,parameter] = NewtonMethod(initialparameter,XInitial,finalstate,MAXIter,eps,elarge)

%NEWTONMETHOD Summary of this function goes here
%   Detailed explanation goes here
%initialparameter=initialparameter';
state_error_weight=[50 50 50 50]; % 设置状态误差的权重，判断是否满足收敛条件

for k=1:MAXIter
   [dhdq,i_h]=FirstDerivatives(initialparameter,XInitial);
   state_error=finalstate-i_h;
   state_error_1=abs(state_error_weight(1)*state_error(1));
   state_error_2=0*abs(state_error_weight(2)*state_error(2));
   state_error_3=abs(state_error_weight(3)*state_error(3));
   state_error_4=abs(state_error_weight(4)*state_error(4));
   if (state_error_1<1)&&(state_error_2<1)&&(norm([state_error_3,state_error_4])<eps)||(norm(i_h)>elarge)
           parameter=initialparameter;
           Q=i_h;
           break;
   end  
    q=initialparameter+(dhdq\(finalstate-i_h)')';%Newton's descent gradient
    parameter=q;
end
end





