clf;        %clears figures
clc;        %clears console
clear;      %clears workspace
close all
axis equal; %keeps the x and y scale the same
%map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];  %default map
%map=[0,0;60,0;60,50;100,50;70,0;110,0;150,80;30,80;30,40;0,80]; %long map
map = [-30,0;-30,40;30,40;30,60;5,60;45,90;85,60;60,60;60,40;120,40;120,60;95,60;135,90;175,60;150,60;150,40;210,40;210,60;185,60;225,90;265,60;240,60;240,40;300,40;300,0];
%map = [0,0;60,0;60,45;45,45];

botSim = BotSim(map,[5,0.005,0.02]);  %sets up a botSim object a map, and debug mode on.
readings = 16;
botSim.setScanConfig(botSim.generateScanConfig(readings));
botSim.drawMap();
drawnow;
botSim.randomPose(10); %puts the robot in a random position at least 10cm away from a wall
botSim.setBotPos([50 40]);
%botSim.setBotPos([20 20]);     
%botSim.setBotPos([-25 20]);
%botSim.setBotPos([-20 30]);
target = botSim.getRndPtInMap(10);  %gets random target.
%target = [10 5];
%target = [295 30];
%target = [80 10];

%starts timer
tic

dist = 6;
stepsize = 5;
[reduced, flag_cw]= reduce_graph(map,dist);

out = visbil_full(reduced,botSim,target, flag_cw);
adjacency = visbil_point_reduced(reduced,target,out,dist,map);

[Noise_level, mean_vec] = s_noise_detector(botSim);

%initialize particles and map objects
modifiedMap = map; 
botSim.setMap(modifiedMap);
max_dim = max([max(map(:,1))-min(map(:,1)) max(map(:,2))-min(map(:,2))]);
num = round(450/330*max_dim);
particles(num,1) = BotSim; 
chosen_particles(num,1) = BotSim(modifiedMap);   
final_particles(num,1) = BotSim(modifiedMap);
mean_particle = BotSim(map);
mean_particle.setScanConfig(botSim.generateScanConfig(readings));

for i=1:num
    particles(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
    chosen_particles(i) = BotSim(modifiedMap);
    chosen_particles(i).setScanConfig(botSim.generateScanConfig(readings));
    particles(i).randomPose(5); %spawn the particles in random locations
    particles(i).setScanConfig(botSim.generateScanConfig(readings));
end

%localisation code
[returnedBot, particles, mean_pos mean_ang] = localisation(botSim,mean_particle,chosen_particles,particles,map,target,readings, Noise_level,num,0,max_dim,[],[],[],0,stepsize,adjacency,reduced,dist);

%reintialize lower number of particles for better performance
num = round(num/3);
for i=1:num
        final_particles(i) = BotSim(modifiedMap);
        final_particles(i).setScanConfig(botSim.generateScanConfig(readings));
        
        final_particles(i).setBotPos(mean_particle.getBotPos + [normrnd(0,0.01) normrnd(0,0.01)]);
        final_particles(i).setBotAng(mean_particle.getBotAng + normrnd(0,4/180*pi));
        
        chosen_particles(i) = BotSim(modifiedMap);
        chosen_particles(i).setScanConfig(botSim.generateScanConfig(readings));
 end
%path planning
[moveCmd turnCmd optimal] = Astar_visbil(mean_pos,mean_ang, adjacency,reduced,botSim,stepsize, target, dist,map);

%call final localisation with less particles
[returnedBot, particles, mean_pos mean_ang] = localisation(botSim,returnedBot,chosen_particles,final_particles,map,target,readings, Noise_level,num,1,max_dim,moveCmd,turnCmd,optimal,1,stepsize,adjacency,reduced,dist);

%Print target and final robot estimate location
position = returnedBot.getBotPos();
disp(['Robot Estimate: [x y angle]: [' num2str(position(1), 3) ' ' num2str(position(2), 3) ' ' num2str(returnedBot.getBotAng(), 5) ']' ]);

returnedBot.drawBot(60,'m')
botSim.drawBot(60,'r')

plot(target(:,1),target(:,2),'x','MarkerSize', 40);
drawnow

resultsTime = toc
%calculated how far away your robot is from the target.
resultsDis =  distance(target, botSim.getBotPos())