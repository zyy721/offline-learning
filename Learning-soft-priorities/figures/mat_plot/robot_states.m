filename = '/home/zhou/lee/safeLearningPriorities/mat_plot/robot_state/robotStates.txt';
delimiter = ' ';
formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';
fileID = fopen(filename,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true, 'TextType', 'string',  'ReturnOnError', false);
fclose(fileID);
robotStates = table(dataArray{1:end-1}, 'VariableNames', {'time','alpha1','alpha2','alpha3','a1','a2','a3','a4','a5','a6','q1','q2','q3','q4','q5','q6'});
clearvars filename delimiter formatSpec fileID dataArray ans;



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


%%%%%%% fitness %%%%%%%%%%%%%
i_cbo = 0:5:800;
f_cbo = -f_cbo - 0.3;
f_cbo_temp = [];
for i=1:1:10
    append_size = size(i_cbo) - size(f_cbo(1,:));
    a = [f_cbo(i, :), f_cbo(i, end)*ones(1, append_size(2))];
    f_cbo_temp = [f_cbo_temp; a];
end
f_cbo = f_cbo_temp;

figure();
shadedErrorBar(i_cbo, f_cbo, {@mean,@std}, 'lineprops', '-r','transparent',true,'patchSaturation',0.075)
hold on
plot([0,800],[0,0],'LineWidth',2)
hold on
axis([0 800 -0.1 0.5]);
legend('fitness value','constraints violations');
xlabel('generations');
y = ylabel('fitness value and constraints violations');
set(y,'rotation',0,'position',[400,0.52],'VerticalAlignment','middle')
set(gcf, 'Color', 'w');
box on
print fitness_value.eps -depsc2 -r600
hold off;
%%%%%%%%% fitness %%%%%%%%%%%%%%%

%%%%%%%%% task priorities %%%%%%%
figure();
plot(robotStates.time,robotStates.alpha1,'r',robotStates.time,robotStates.alpha2,'g',robotStates.time,robotStates.alpha3,'b','LineWidth',2);
ax = gca;
ax.GridLineStyle = '--';
ax.LineWidth = 1.0;
hold on

legend('\alpha_1','\alpha_2','\alpha_3','Location','southeast');
axis([0 robotStates.time(end) -0.2 1.2]);
xlabel('time [s]');
y = ylabel('task priorities');
set(y,'rotation',0,'position',[13,1.28],'VerticalAlignment','middle')
set(gcf, 'Color', 'w');
print task_priorities.eps -depsc2 -r600
hold off;
%%%%%%%%% task priorities %%%%%%%

%%%%%%%%% joint command %%%%%%%%%
figure();
plot(robotStates.time,robotStates.a1,robotStates.time,robotStates.a2,robotStates.time,robotStates.a3,robotStates.time,robotStates.a4,robotStates.time,robotStates.a5,robotStates.time,robotStates.a6,'LineWidth',2);
ax = gca;
ax.GridLineStyle = '--';
ax.LineWidth = 1.0;
hold on

legend('u_1','u_2','u_3','u_4','u_5','u_6','Location','southeast');
axis([0 robotStates.time(end) -0.4 0.2]);
xlabel('time [s]');
y = ylabel('joint accelerations [m/s^2]');

set(y,'rotation',0,'position',[13,0.23],'VerticalAlignment','middle')

set(gcf, 'Color', 'w');
print joint_accelerations.eps -depsc2 -r600
hold off;
%%%%%%%%% joint command %%%%%%%%%
