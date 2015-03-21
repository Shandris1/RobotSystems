clf;        %clears figures
clc;        %clears console
clear;      %clears workspace
close all

robotStart();

axis equal; %keeps the x and y scale the same
map=[0,0;66,0;66,44;44,44;44,66;110,66;110,110;0,110];  %default map
%map=[0,0;60,0;60,50;100,50;70,0;110,0;150,80;30,80;30,40;0,80]; %long map
%map = [-30,0;-30,40;30,40;30,60;5,60;45,90;85,60;60,60;60,40;120,40;120,60;95,60;135,90;175,60;150,60;150,40;210,40;210,60;185,60;225,90;265,60;240,60;240,40;300,40;300,0];
%test = [];

%botSim = BotSim(map,[0.01,0.005,0]);  %sets up a botSim object a map, and debug mode on.
%botSim = BotSim(map,[0.1,0.1,0.01]);  %sets up a botSim object a map, and debug mode on.
botSim = BotSim(map,[2,0,0]);
readings = 20;
Scan_number = readings;

botSim.drawMap();
%drawnow;
%botSim.setBotPos([110 62]);
%botSim.setBotAng(pi/2);
%target = botSim.getRndPtInMap(10);  %gets random target.
target = [23 20];
%plot(target(:,1),target(:,2),'x');
%starts timer
tic
dist = 12;
stepsize = 10;

[reduced, flag_cw]= reduce_graph(map,dist);
out = visbil_full(reduced,botSim,target, flag_cw);
adjacency =visbil_point_reduced(reduced,target,out,dist,map);
Noise_level = 10;
speed = 100;
botSim.setScanConfig(botSim.generateScanConfig(readings));

%initialize particles and map objects
modifiedMap = map; 
botSim.setMap(modifiedMap);
max_dim = [max([max(map(:,1))-min(map(:,1)) max(map(:,2))-min(map(:,2))])  max(map(:,1))-min(map(:,1)) max(map(:,2))-min(map(:,2))];
num = round(450/330*max_dim(1));
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
    %index_initial = round(rand(1)*(size(workspace,1)-1))+1;
    %particles(i).setBotPos([workspace(index_initial,:)]);
    %particles(i).setBotAng(2*pi*rand(1));
    %workspace(index_initial,:) = [];
    particles(i).setScanConfig(botSim.generateScanConfig(readings));
end

%your localisation function is called here.
[returnedBot, particles, mean_pos mean_ang] = localise_final3_real(botSim,mean_particle,chosen_particles,particles,map,target,readings, Noise_level,num,max_dim,0,speed); %Where the magic happens

%reintialize particles and change num
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

converged = 0;

while(converged == 0)
    
    [mean_particle] = movement(num,final_particles,returnedBot,moveCmd,turnCmd,optimal,speed);

    %check if you reached the target 
    if(distance(target, mean_particle.getBotPos()) <= 5)
        converged = 1;
    else
        %your localisation function is called here.
        [returnedBot, final_particles, mean_pos mean_ang] = localise_final3_real(botSim,mean_particle,chosen_particles,final_particles,map,target,readings, Noise_level,num,max_dim,1,speed);

        %path planning
        [moveCmd turnCmd optimal] = Astar_visbil(mean_pos,mean_ang, adjacency,reduced,botSim,stepsize, target, dist,map);
    end

end
% %call final localisation with less particles
% [returnedBot, particles, mean_pos mean_ang] = localise_final3_real(botSim,returnedBot,chosen_particles,final_particles,map,target,readings, Noise_level,num,1,max_dim,moveCmd,turnCmd,optimal,1,stepsize,adjacency,reduced,dist);
% 


%Print target and final robot estimate location
disp(['Robot Estimate [x y angle]: ' num2str(mean_pos(1), 3) ' ' num2str(mean_pos(2), 3) ' ' num2str(mean_ang, 5)]);

returnedBot.drawBot(30,'m')
resultsTime = toc
%calculated how far away your robot is from the target.
resultsDis =  distance(target, botSim.getBotPos())
