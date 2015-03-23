clf;        %clears figures
clc;        %clears console
clear;      %clears workspace
close all

%initalizes and reset NXT, open sensors and reset sensor encoders
robotStart();

axis equal; %keeps the x and y scale the same
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%map goes here
map=[0,0;66,0;66,44;44,44;44,66;110,66;110,110;0,110];  %default map
%map=[0,0;60,0;60,50;100,50;70,0;110,0;150,80;30,80;30,40;0,80]; %long map
%map = [-30,0;-30,40;30,40;30,60;5,60;45,90;85,60;60,60;60,40;120,40;120,60;95,60;135,90;175,60;150,60;150,40;210,40;210,60;185,60;225,90;265,60;240,60;240,40;300,40;300,0];
%test = [];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%initalize botsim object
%botSim = BotSim(map,[0.01,0.005,0]);  %sets up a botSim object a map, and debug mode on.
%botSim = BotSim(map,[0.1,0.1,0.01]);  %sets up a botSim object a map, and debug mode on.
botSim = BotSim(map,[2,0,0]);
botSim.drawMap();

%user changeable inputs
readings = 20;
target = [88 90];
dist = 15;
stepsize = 15;
Noise_level = 10;
cloudsize = 0.01;
cloudangle = 4/180*pi;

%starts timer
tic

%constructs configuration space
[reduced, flag_cw]= reduce_graph_robot(map,dist);
%creates visibility graph of the map
out = visbil_full(reduced,botSim,target, flag_cw);
%creates visibility graph of the target
adjacency = visbil_point_reduced(reduced,target,out,dist,map);

%set unique map and calculate max dimensions 
[map] = unique(map,'rows','stable');
botSim.setMap(map);
max_dim = [max([max(map(:,1))-min(map(:,1)) max(map(:,2))-min(map(:,2))])  max(map(:,1))-min(map(:,1)) max(map(:,2))-min(map(:,2))];
num = round(650/330*max_dim(1));

%initialize particles 
particles(num,1) = BotSim; 
chosen_particles(num,1) = BotSim(map);   
final_particles(num,1) = BotSim(map);

mean_particle = BotSim(map);
mean_particle.setScanConfig(botSim.generateScanConfig(readings));

for i=1:num
    particles(i) = BotSim(map);  
    
    %used for resampled particles
    chosen_particles(i) = BotSim(map);
    chosen_particles(i).setScanConfig(botSim.generateScanConfig(readings));
    
    %spawn the particles in random locations
    particles(i).randomPose(10); 
    particles(i).setScanConfig(botSim.generateScanConfig(readings));
end

%localise function
[returnedBot, particles, mean_pos mean_ang, botScan] = real_robot_localisation(botSim,mean_particle,chosen_particles,particles,map,target,readings, Noise_level,num,0,max_dim,[],[],[],0,stepsize,adjacency,reduced,dist,[]); %Where the magic happens

%reintialize particles and change num
num = round(num/3);
for i=1:num
        %TODO check mean particle output
        final_particles(i) = BotSim(map);
        final_particles(i).setScanConfig(botSim.generateScanConfig(readings));
        
        final_particles(i).setBotPos(mean_particle.getBotPos + [normrnd(0,cloudsize) normrnd(0,cloudsize)]);
        final_particles(i).setBotAng(mean_particle.getBotAng + normrnd(0,cloudangle));
        
        %used for resampled particles
        chosen_particles(i) = BotSim(map);
        chosen_particles(i).setScanConfig(botSim.generateScanConfig(readings));
 end

%path planning - construct visibility graph of current position
%perform a star
[moveCmd turnCmd optimal] = Astar_visbil(mean_pos,mean_ang, adjacency,reduced,botSim,stepsize, target, dist,map);

%call tracking with less particles
[returnedBot, particles, mean_pos mean_ang botScan] = real_robot_localisation(botSim,returnedBot,chosen_particles,final_particles,map,target,readings, Noise_level,num,1,max_dim,moveCmd,turnCmd,optimal,1,stepsize,adjacency,reduced,dist,botScan);

%Print target and final robot estimate location
disp(['Robot Estimate [x y angle]: [' num2str(mean_pos(1), 3) ' ' num2str(mean_pos(2), 3) ' ' num2str(mean_ang, 5) ']']);

returnedBot.drawBot(30,'m');
resultsTime = toc
%calculated how far away your robot is from the target.
resultsDis =  distance(target, botSim.getBotPos())
