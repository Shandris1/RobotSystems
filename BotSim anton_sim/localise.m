function [botSim] = localise(botSim,map,target)

axis equal;

%starts timer
tic

dist = 6;
stepsize = 5;
[reduced, flag_cw]= reduce_graph(map,dist);

out = visbil_full(reduced,botSim,target, flag_cw);
adjacency = visbil_point_reduced(reduced,target,out,dist,map);

readings = 16;
botSim.setScanConfig(botSim.generateScanConfig(readings));

%check for noise
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
[moveCmd turnCmd optimal] = Astar_visbil(mean_pos,mean_ang, adjacency,reduced,botSim,stepsize,target,dist,map);

%call final localisation with less particles
[returnedBot, particles, mean_pos mean_ang] = localisation(botSim,returnedBot,chosen_particles,final_particles,map,target,readings, Noise_level,num,1,max_dim,moveCmd,turnCmd,optimal,1,stepsize,adjacency,reduced,dist);

%Print target and final robot estimate location
position = returnedBot.getBotPos();
disp(['Robot Estimate: [x y angle]: [' num2str(position(1), 3) ' ' num2str(position(2), 3) ' ' num2str(returnedBot.getBotAng(), 5) ']' ]);

%stops timer
toc;