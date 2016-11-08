function FigureImprovement(inputfigure)
%%%%%%%%%%%%%%%%%%%Figure Setup%%%%%%%%%%%%%%%%
width = 3;     % Width in inches
height = 3;    % Height in inches
alw = 0.75;    % AxesLineWidth
fsz = 11;      % Fontsize
lw = 1.5;      % LineWidth
msz = 8;       % MarkerSize

pos = get(gcf, 'Position');
%set(gcf, 'Position', [pos(1) pos(2) width*100, height*100]); %<- Set size
set(gca, 'FontSize', fsz, 'LineWidth', alw); %<- Set properties
name=inputfigure.Name;
figure(inputfigure);
%%%%%%%%%%%%%%%%%%%%%%%%%Special for 'sim'%%%%%%%
if strcmp(name,'Simulation')==1
    %axis([-1 31 -5 5]);%Straight-Line
    axis([29 84 -10 35]);%Curve-Line
    axis off;
    grid off;
    print('Simulation','-dpng','-r300');
    print('Simulation','-depsc','-r300');
    %%%%%%%%%%%%%%%%%%%%%%%%%Special for 'curvatur'%%%%%%%
else if strcmp(name,'Curvature')==1
         axis([0 31 -0.4 0.4]);
         %print('Curvature','-dpng','-r300');
         %print('Curvature','-depsc','-r300');
    end
end



end