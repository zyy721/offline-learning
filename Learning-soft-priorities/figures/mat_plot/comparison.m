clear
clc
f_cma = [];
for i=1:1:10
    filename = ['/home/zhou/lee/safeLearningPriorities/mat_plot/cmaes_fitness/cmaes' num2str(i) '.dat'];
    delimiter = ' ';
    formatSpec = '%*s%f%*s%*s%f%*s%*s%*s%[^\n\r]';
    fileID = fopen(filename,'r');
    dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true, 'TextType', 'string',  'ReturnOnError', false);
    fclose(fileID);
%     name_string = ['f_cma' num2str(i) '=dataArray{:, 2}'];
%     eval(name_string)
    a = dataArray{:, 2};
    f_cma = [f_cma;a'];
    clearvars filename delimiter formatSpec fileID dataArray ans;
end
i_cma = (12:12:804)';
i_cma(1)=0;
i_cma(end) = 800;
f_cma = f_cma - 0.3;
%%%%%%%%%%%%%%%% cbo %%%%%%%%%%%%%%%%%
f_cbo = [];
for i=1:1:10
    filename = ['/home/zhou/lee/safeLearningPriorities/mat_plot/cbo_fitness/cbo' num2str(i) '.txt'];
    delimiter = ' ';
    formatSpec = '%f%f%[^\n\r]';
    fileID = fopen(filename,'r');
    dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true, 'TextType', 'string',  'ReturnOnError', false);
    fclose(fileID);
    a = dataArray{:, 2};
    f_cbo = [f_cbo;a'];
    clearvars filename d
    clearvars filename delimiter formatSpec fileID dataArray ans;
end

i_cbo = 0:5:800;
f_cbo = -f_cbo - 0.3;
f_cbo_temp = [];
for i=1:1:10
    append_size = size(i_cbo) - size(f_cbo(1,:));
    a = [f_cbo(i, :), f_cbo(i, end)*ones(1, append_size(2))];
    f_cbo_temp = [f_cbo_temp; a];
end
f_cbo = f_cbo_temp;

% i_bo = vertcat(i_bo, (260:5:800)');
% append_size = size((260:5:800)');
% f_bo = [f_bo; f_bo(end)*ones(append_size(1),1)];
%%%%%%%%%%%%%%%% cbo %%%%%%%%%%%%%%%%%



figure();


% figure();
% for i=1:1:10
%     name_string = ['plot(i_cma,f_cma' num2str(i) ')'];
%     eval(name_string)
%     hold on
% end

shadedErrorBar(i_cma, f_cma, {@mean,@std}, 'lineprops', '-b','transparent',true,'patchSaturation',0.075)
hold on

shadedErrorBar(i_cbo, f_cbo, {@mean,@std}, 'lineprops', '-r','transparent',true,'patchSaturation',0.075)
hold on


plot([0,800],[0,0],'LineWidth',2)
hold on

axis([0 800 -0.1 0.5]);
legend('BO with Constraints','CMA-ES with Constraints','constraints violations');
xlabel('generations');
y = ylabel('fitness value and constraints violations');

box on

set(y,'rotation',0,'position',[400,0.52],'VerticalAlignment','middle')
set(gcf, 'Color', 'w');
print cmaesVScbo.eps -depsc2 -r600
hold off;
