%--------------------------------------------------
%   子函数：用来绘制置信区域（椭圆或椭球）
%   Pay attention:P is the inverse matrix of coviance matrix!
%--------------------------------------------------
function  h = ellipsefig(xc,P,r,tag)
% 画一般椭圆或椭球：(x-xc)'*P*(x-xc) = r
[V, D] = eig(P); %P是逆矩阵，所以aa是1/D(1),bb是1/D(4)
if tag == 1
    aa = sqrt(r/D(1));
    bb = sqrt(r/D(4));
    t = linspace(0, 2*pi, 60);
    xy = V*[aa*cos(t);bb*sin(t)]; % 坐标旋转
    h = plot(xy(1,:)+xc(1),xy(2,:)+xc(2), 'b-', 'linewidth', 0.2);
else
    aa = sqrt(r/D(1,1));
    bb = sqrt(r/D(2,2));
    cc = sqrt(r/D(3,3));
    [u,v] = meshgrid(linspace(-pi,pi,30),linspace(0,2*pi,30));
    x = aa*cos(u).*cos(v);
    y = bb*cos(u).*sin(v);
    z = cc*sin(u);
    xyz = V*[x(:)';y(:)';z(:)'];  % 坐标旋转
    x = reshape(xyz(1,:),size(x))+xc(1);
    y = reshape(xyz(2,:),size(y))+xc(2);
    z = reshape(xyz(3,:),size(z))+xc(3);
    h = mesh(x,y,z);  % 绘制椭球面网格图
end