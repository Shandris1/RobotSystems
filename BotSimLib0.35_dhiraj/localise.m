function [botSim] = localise(botSim,map,target)

axis equal;

%starts timer
tic

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

%reintialize particles and change num
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

%stops timer
toc;