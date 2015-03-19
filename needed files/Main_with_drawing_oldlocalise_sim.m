clf;        %clears figures
clc;        %clears console
clear;      %clears workspace
close all
axis equal; %keeps the x and y scale the same
%map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];  %default map
%map=[0,0;60,0;60,50;100,50;70,0;110,0;150,80;30,80;30,40;0,80]; %long map
%map = [-30,0;-30,40;30,40;30,60;5,60;45,90;85,60;60,60;60,40;120,40;120,60;95,60;135,90;175,60;150,60;150,40;210,40;210,60;185,60;225,90;265,60;240,60;240,40;300,40;300,0];
%map = [0,0;60,0;60,45;45,45];
map = [-30,0;-30,40;30,40;5,60;45,90;85,60;60,40;120,40;95,60;135,90;175,60;150,40;210,40;185,60;225,90;265,60;240,40;300,40;300,0];
%map = [-30,0;-30,40;30,40;30,45;5,45;5,85;85,85;85,45;60,45;60,40;120,40;120,45;95,45;135,65;175,45;150,45;150,40;210,40;210,45;185,45;225,65;265,45;240,45;240,40;300,40;300,0];
%map = [-30,0;-30,40;30,40;30,45;5,45;45,65;85,45;60,45;60,40;120,40;120,45;95,45;135,65;175,45;150,45;150,40;210,40;210,45;185,45;225,65;360,45;240,45;240,40;300,40;300,0];
%map=[0,0;60,0;60,50;100,50;70,0;110,0;150,80;30,80;30,40;0,80]; %long map
%map = [0,0; -30,-30; -25,-25; 30,-25; 30, 0;100,0;100,45;45,45];

botSim = BotSim(map,[0,0,0]);  %sets up a botSim object a map, and debug mode on.
readings = 16;
botSim.setScanConfig(botSim.generateScanConfig(readings));
botSim.drawMap();
drawnow;
botSim.randomPose(0); %puts the robot in a random position at least 10cm away from a wall
%botSim.setBotPos([115 65]);
%botSim.setBotPos([20 20]);     
%botSim.setBotPos([-25 20]);
%botSim.setBotPos([-20 30]);
target = botSim.getRndPtInMap(10);  %gets random target.
%target = [295 30];
target = [50 50];
[Noise_level, mean_vec] = s_noise_detector(botSim);
%starts timer
tic

%initialize particles and map objects
modifiedMap = map; 
botSim.setMap(modifiedMap);
max_dim = max([max(map(:,1))-min(map(:,1)) max(map(:,2))-min(map(:,2))]);
num = round(450/330*max_dim);
particles(num,1) = BotSim; 
chosen_particles(num,1) = BotSim(modifiedMap);   
final_particles(num,1) = BotSim(modifiedMap);
for i=1:num
    particles(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
    chosen_particles(i) = BotSim(modifiedMap);
    chosen_particles(i).setScanConfig(botSim.generateScanConfig(readings));
    particles(i).randomPose(5); %spawn the particles in random locations
    %index_initial = round(rand(1)*(size(workspace,1)-1))+1;
    %particles(i).setBotPos([workspace(index_initial,:)]);
    %particles(i).setBotAng(2*pi*rand(1));
    %workspace(index_initial,:) = [];
    particles(i).setScanConfig(botSim.generateScanConfig(readings));
end

%localisation code
[returnedBot, particles, mean_pos mean_ang] = localisation(botSim,chosen_particles,particles,map,target,readings, Noise_level,num,0,max_dim,[],[],[],0); %Where the magic happens

%reintialize lower number of particles for better performance
num = round(num/3);
for i=1:num
    final_particles(i) = BotSim(modifiedMap);
    final_particles(i).setScanConfig(botSim.generateScanConfig(readings));
    
    final_particles(i).setBotPos(particles(i).getBotPos());
    final_particles(i).setBotAng(particles(i).getBotAng());
    
    chosen_particles(i) = BotSim(modifiedMap);
    chosen_particles(i).setScanConfig(botSim.generateScanConfig(readings));
end

%path planning
[moveCmd turnCmd optimal] = pathPlan(botSim,map,target,mean_pos,mean_ang);

%call final localisation with less particles
[returnedBot, particles, mean_pos mean_ang] = localisation(botSim,chosen_particles,final_particles,map,target,readings, Noise_level,num,1,max_dim,moveCmd,turnCmd,optimal,1);

%Print target and final robot estimate location
fprintf('Target: %.3f %.3f, RobotEstimate[x y angle]: %.3f %.3f %.3f',target,[ mean_pos(:,1), mean_pos(:,2), mean_ang ]);

returnedBot.drawBot(60,'m')
plot(target(:,1),target(:,2),'x','MarkerSize', 40);
drawnow

resultsTime = toc
%calculated how far away your robot is from the target.
resultsDis =  distance(target, returnedBot.getBotPos())