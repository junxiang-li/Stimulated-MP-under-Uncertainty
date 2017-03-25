clc;clear;
t=0:0.01:10;
lamda=4.3429;
y1=exp(-t);
y2=exp(-t/lamda);
plot(t,y1,'r');hold on;
plot(t,y2,'b');
%axis equal;
grid on;
set(gca,'XTick',[0:0.5:10]);
set(gca,'YTick',[0:0.1:1]);
% clc;clear;
% parameter=ones(4,1);
% parameter(1)=parameter_lookup.trajectory_parameter(1,2,1);
% parameter(2)=parameter_lookup.trajectory_parameter(1,2,2);
% parameter(3)=parameter_lookup.trajectory_parameter(1,2,3);
% parameter(4)=parameter_lookup.trajectory_parameter(1,2,4);

% load full_tree.mat
% A=lookup_table;
% parameter=ones(4,1);
% parameter=lookup_table(1,1).ParaMatrix(:,4);

% if U(:,i)~= Inf & 1==1
%     b=0;
% else U(:,i)== Inf
%     b=1;
% end

% clear all
% clc
% a=[0, 0];
% b=[-100 ,100];
% c=[-100,100];
% d=[0 ,0];
% figure(1)
% plot(a,b,'k','LineWidth',10)
% hold on
% plot(c,d,'k','LineWidth',10)
% hold on
% x1=[-100,0];
% x2=[0 -80];
% 
% v=1;
% aa=[];
% ba=[];
% for i=1:180;
% aa(i)=-100+v.*i;
% drawnow;
% h1=plot(aa(i),0,'-or','LineWidth',5);
% pause(0.01) ;
% 
% bb(i)=-80+v.*i;
% drawnow;
% h2=plot(0,bb(i),'-ob','LineWidth',5);
% pause(0.01) ;
% 
% delete(h1);
% delete(h2);
% 
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Plot Vehicle Rectangle%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% clc;clear;
% point=[3,5];
% rotate_theta=pi/2;
% vehiclewidth=1;
% vehiclelength=2;
% endpoints_rightfront=point+[vehiclelength,-vehiclewidth]*[cos(rotate_theta),sin(rotate_theta);-sin(rotate_theta),cos(rotate_theta)];
% endpoints_rightrear=point+[-vehiclelength,-vehiclewidth]*[cos(rotate_theta),sin(rotate_theta);-sin(rotate_theta),cos(rotate_theta)];
% endpoints_leftfront=point+[vehiclelength,vehiclewidth]*[cos(rotate_theta),sin(rotate_theta);-sin(rotate_theta),cos(rotate_theta)];
% endpoints_leftrear=point+[-vehiclelength,vehiclewidth]*[cos(rotate_theta),sin(rotate_theta);-sin(rotate_theta),cos(rotate_theta)];
% rect=[endpoints_leftrear;endpoints_leftfront;endpoints_rightfront;endpoints_rightrear;endpoints_leftrear];
% figSim=figure('NumberTitle','off','Name','Simulation');
% plot(rect(:,1), rect(:,2),'k-');hold on;
% figCur=figure('NumberTitle','off','Name','Curvature');
% plot(rect(:,1), rect(:,2),'b--');hold on;
% axis square;axis([-10,10,-10,10]);
% figure(figSim);
% fill(rect(:,1), rect(:,2),'r');
% axis square;axis([-10,10,-10,10]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Dynamically Plot Vehicle Rectangle%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% clf
% for i = 0:pi/12:20*pi,
%     p1=[cos(i),sin(i)];
%     p2=[cos(i+pi/2),sin(i+pi/2)];
%     p3=[cos(i+pi),sin(i+pi)];
%     p4=[cos(i+pi/2*3),sin(i+pi/2*3)];
%     rect=[p1;p2;p3;p4;p1];
%     %cla;
%     fill(rect(:,1),rect(:,2),'b');
%     axis([-5 5 -5 5]);
%     axis equal;
%     drawnow; 
%     pause(0.1);
% end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Plot Velocity Profile%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% clc;clear;
% 
% % x=0:11;y=[2 2.5 3.5 4 4 4 4 3.5 2.5 1.5 0.5 0];
% % cs = spline(x,[0 y 0]);
% % plot(x,ppval(cs,x),'-.','linewidth', 2);hold on;
% 
% 
% x=0:4;y=[2 2.3 3 3.7 4];
% cs = spline(x,y);
% xx = linspace(0,4,100);
% plot(xx,ppval(cs,xx),'-','linewidth', 1,'color','r');hold on;
% plot(0, 2, 'k.', 'markersize', 20);
% 
% x=4:0.5:6;
% y=linspace(4,4,5);
% plot(x,y,'--','linewidth',1.5,'color','magenta');
% plot(4, 4, 'k.', 'markersize', 20);
% plot(6, 4, 'k.', 'markersize', 20);
% 
% x=6:11;y= [4 3.5 2.5 1.5 0.5 0];
% cs = spline(x,y);
% xx = linspace(6,11,200);
% plot(xx,ppval(cs,xx),'-.','linewidth', 1);hold on;
% plot(11, 0, 'k.', 'markersize', 20);
% 
% % x=[0 3.5 6 10];
% %     y=[2 4 4  0.5];
% %     plot(x,y,'o');
% % 
% % set(gca,'ytick',[]);
% % set(gca,'xtick',[]);
% % annotation('arrow',[0.13 0.13],[0.8 1]);
% % annotation('arrow',[0.8 1],[0.109 0.109]);
% 
% 
% % Some bogus functions
% % f = @(x) 50* 1.6.^(-x-5);
% % g = @(x) 50* 1.6.^(+x-10);
% 
% % % Point where they meet
% % xE = 2.5;
% % yE = f(xE);
% 
% % Plot the bogus functions
% % figure(1), clf, hold on
% % x = 0:0.2:5;
% % plot(x,f(x),'r',  x,g(x),'b', 'linewidth', 2)
% 
% % get rid of standard axes decorations
% set(gca, 'Xtick', [], 'Ytick', [], 'box', 'off')
% 
% % Fix the axes sizes
% %axis([0 5 0 5])
% axis([0 12 0 6])
% % the equilibrium point
% %plot(xE, yE, 'k.', 'markersize', 20)
% 
% % the dashed lines
% %line([xE 0; xE xE], [0 yE; yE yE], 'linestyle', '--', 'color', 'k')
% 
% % the arrows
% xO = 0.2;  
% yO = 0.1;
% patch(...
%     [12-xO -yO; 12-xO +yO; 12 0.0], ...
%     [yO 6-xO; -yO 6-xO; 0 6], 'k', 'clipping', 'off')
% 
% % the squishy wiggly line pointing to the "equilibrium" text
% % h = @(x)0.5*(x+0.2) + 0.1*sin((x+0.2)*14);
% % x = 2.7:0.01:3.5;
% % plot(x, h(x), 'k', 'linewidth', 2)
% 
% % the static texts
% text(0, 1.8, '(v_0,a_0)', 'fontweight', 'bold','fontsize',18);
% text(3.8,   4.2, '(v_1,a_1)', 'fontweight', 'bold','fontsize',18);
% text(5.8,   4.2, '(v_2,a_2)', 'fontweight', 'bold','fontsize',18);
% text(11,   0.2, '(v_3,a_3)', 'fontweight', 'bold','fontsize',18);
% textstr={'Ramp-up '; 'velocity profile'};
% text(3,    2.5, textstr, 'rotation', 0, 'fontsize', 16);
% textstr={'Ramp-down '; 'velocity profile'};
% text(8.5,   3.5,textstr, 0, 'fontsize', 16);
% 
% text(-0.2,-0.2,'0','fontsize',18);
% text(    12, -0.2, 't', 'fontsize', 18);
% text(-4*yO,    6, 'v', 'rotation', 0, 'fontsize', 18);
% 
% %annotation('arrow',[0.8 1],[0.109 0.109]);
% % text(   .5,  4.2, 'Demand', 'fontsize', 14, 'rotation', -55)
% % text(   4.0,  3.3, 'Supply', 'fontsize', 14, 'rotation', +55)
% % text(   3.6,  2.1, 'Equilibrium', 'fontsize', 14)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Plot vdesire Function%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% clc;clear;
% 
% %Defaults for this blog post
% width = 3;     % Width in inches
% height = 3;    % Height in inches
% alw = 0.75;    % AxesLineWidth
% fsz = 18;      % Fontsize
% lw = 1.5;      % LineWidth
% msz = 8;       % MarkerSize
% 
% %Setup for the profile
% V_max=10;%unit:m/s
% D_safe=2;%unit:m
% proportion_two_part=4;%1:4
% 
% % Some bogus functions
% f = @(x) 2*V_max./(1+exp(D_safe-x))-V_max;
% % Plot the bogus functions
% figure(1), clf, hold on;
% pos = get(gcf, 'Position');
% set(gcf, 'Position', [pos(1) pos(2) width*1000, height*1000]); %<- Set size
% set(gca, 'FontSize', fsz, 'LineWidth', alw); %<- Set properties
% 
% x = 0: 0.2:D_safe;
% g = zeros(length(x));
% plot(x,g,'b', 'linewidth', lw);
% x = D_safe:0.2:proportion_two_part*D_safe;
% plot(x,f(x),'r', 'linewidth', lw);
% 
% %  Point where they meet
% plot(D_safe, 0, 'k.', 'markersize', 20);
% 
% % get rid of standard axes decorations
% set(gca, 'Xtick', [], 'Ytick', [], 'box', 'off')
% 
% %Fix the axes sizes
% x_axis=8;
% y_axis=12;
% axis([0 x_axis 0 y_axis]);
% 
% % the dashed lines
% x_temp=[0:proportion_two_part*D_safe]
% plot(x_temp,V_max*ones(length(x_temp)), 'linestyle', '--', 'color', 'k' );
% 
% % the arrows
% xO = 0.2;  
% yO = 0.1;
% patch(...
%     [x_axis-xO -yO; x_axis-xO +yO; x_axis 0.0], ...
%     [yO y_axis-xO; -yO y_axis-xO; 0 y_axis], 'k', 'clipping', 'off');
% %axis texture
% text(-0.2,-0.2,'0','fontsize',18);
% text(    x_axis, -0.2, '$$ s $$','Interpreter','latex', 'fontsize', 18);
% text(-4*yO-0.3,    y_axis+0.3, '$$V_{desire}$$','Interpreter','latex', 'rotation', 0, 'fontsize', fsz);
% 
% 
% % the static texts
% text(-4*yO-0.3, 10, '$$ V_{max} $$','Interpreter','latex', 'fontweight', 'bold','fontsize',fsz);
% text(D_safe,   -0.3, '$$ D_{safe} $$','Interpreter','latex', 'fontweight', 'bold','fontsize',fsz);
% annotation('textarrow',[0.52 0.6],[0.62 0.5]);
% %annotation('arrow',[0.48 0.55],[0.64 0.5]);
% textstr='$$ \frac{2V_{max}}{1+e^{(D_{safe}-s)}}-V_{max} $$';
% text(4,    4.8, textstr,'Interpreter','latex', 'rotation', 0, 'fontsize', fsz);
% print('desirevelocity','-depsc','-r300');

% %print('velocityprofile','-dpng','-r300');
% % if ispc % Use Windows ghostscript call
% %   system('gswin64c -o -q -sDEVICE=png256 -dEPSCrop -r300 -ovelocityprofile_eps.png velocityprofile.eps');
% % else % Use Unix/OSX ghostscript call
% %   system('gs -o -q -sDEVICE=png256 -dEPSCrop -r300 -ovelocityprofile_eps.png velocityprofile.eps');
% % end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Figure Improvement%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% f = @(x) x.^2;
% g = @(x) 5*sin(x)+5;
% dmn = -pi:0.001:pi;
% xeq = dmn(abs(f(dmn) - g(dmn)) < 0.002);
% figure(1);
% plot(dmn,f(dmn),'b-',dmn,g(dmn),'r--',xeq,f(xeq),'g*');
% xlim([-pi pi]);
% legend('f(x)', 'g(x)', 'f(x)=g(x)', 'Location', 'SouthEast');
% xlabel('x');
% title('Example Figure');
% print('example', '-dpng', '-r300'); %<-Save as PNG with 300 DPI

% Defaults for this blog post
% width = 3;     % Width in inches
% height = 3;    % Height in inches
% alw = 0.75;    % AxesLineWidth
% fsz = 11;      % Fontsize
% lw = 1.5;      % LineWidth
% msz = 8;       % MarkerSize
% 
% figure(1);
% pos = get(gcf, 'Position');
% set(gcf, 'Position', [pos(1) pos(2) width*100, height*100]); %<- Set size
% set(gca, 'FontSize', fsz, 'LineWidth', alw); %<- Set properties
% plot(dmn,f(dmn),'b-',dmn, g(dmn),'r--',xeq,f(xeq),'g*','LineWidth',lw,'MarkerSize',msz); %<- Specify plot properites
% xlim([-pi pi]);
% legend('f(x)', 'g(x)', 'f(x)=g(x)', 'Location', 'SouthEast');
% xlabel('x');
% title('Improved Example Figure');
% 
% % Set Tick Marks
% set(gca,'XTick',-3:3);
% set(gca,'YTick',0:10);
% 
% % Here we preserve the size of the image when we save it.
% set(gcf,'InvertHardcopy','on');
% set(gcf,'PaperUnits', 'inches');
% papersize = get(gcf, 'PaperSize');
% left = (papersize(1)- width)/2;
% bottom = (papersize(2)- height)/2;
% myfiguresize = [left, bottom, width, height];
% set(gcf,'PaperPosition', myfiguresize);
% 
% % Save the file as PNG
% print('improvedExample','-dpng','-r300');
%print('improvedExample','-dpdf','-r300');
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%axe Function%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% clc;clear;close all;                                                                                                               
% t=linspace(0,6,300);% 生成曲线的整体离散坐标�?t
% t1=linspace(2.8,3.2,300); % 生成曲线的局部离散坐标�?t1                                                                                                                       
% y=sin(1./[t-3]);% 生成曲线的整体离散函数�?y
% y1=sin(1./[t1-3]); % 生成曲线的局部离散函数�?y1                                                                                                                                  
% figure;% 生成新的图形窗口            
% plot(t,y);axis('equal'); % 绘制整体曲线�?                                                                                             
% axes('Position',[0.18,0.62,0.28,0.25]); % 生成子图                                                                             
% plot(t1,y1); % 绘制�?��曲线�?                                                                                                               
% xlim([min(t1),max(t1)]); % 设置坐标轴范�?