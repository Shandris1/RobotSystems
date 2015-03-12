clf;        %clears figures
clc;        %clears console
clear;      %clears workspace
close all
axis equal; %keeps the x and y scale the same
%map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];  %default map
%map=[0,0;60,0;60,50;100,50;70,0;110,0;150,80;30,80;30,40;0,80]; %long map
map = [-30,0;-30,40;30,40;30,60;5,60;45,90;85,60;60,60;60,40;120,40;120,60;95,60;135,90;175,60;150,60;150,40;210,40;210,60;185,60;225,90;265,60;240,60;240,40;300,40;300,0];
test = [];
movement_step = 3;

% botSim = BotSim(map,[0.01,0.005,0]);  %sets up a botSim object a map, and debug mode on.
%botSim = BotSim(map,[0.1,0.1,0.01]);  %sets up a botSim object a map, and debug mode on.
botSim = BotSim(map,[2,0,0]);
readings = 16
botSim.setScanConfig(botSim.generateScanConfig(readings));

botSim.drawMap();
drawnow;
%for i =1:200
botSim.randomPose(10); %puts the robot in a random position at least 10cm away from a wall
tic
[test] = s_noise_detector(botSim);
resultsTime = toc
%botSim.setBotPos([110 62]);
%botSim.setBotAng(pi/2);
target = botSim.getRndPtInMap(10);  %gets random target.
%target = [80 10];
plot(target(:,1),target(:,2),'x');
 %starts timer
tic
%your localisation function is called here.
[returnedBot pos ang] = localise_final3(botSim,map,target,readings); %Where the magic happens
%close_array = map_obstacle(map,1,1);

out = visbil_full(map,botSim,target);
adjacency = visbil_point(map,target,out);
[large_move turn optimal] = Astar_visbil(pos,ang, adjacency,map,botSim)

%[times_move,remains_move] = quorem(round(large_move),round(movement_step))

%[move turn optimal] = Astar(round(pos),ang,round(target),close_array,1,botSim);
returnedBot.drawBot(30,'m')
currentAng= ang;
for i=1:size(move)
delta =turn(i)- currentAng;
botSim.turn(delta);
botSim.move(move(i));
currentAng = turn(i);
currentPos = botSim.getBotPos();
%x =[currentPos(1) currentPos(1)+cos(currentAng)*3];
%y = [currentPos(2) currentPos(2)+sin(currentAng)*3];
botSim.drawBot(3,'g');
%refreshdata(h1,'caller')
%refreshdata(h2,'caller')
drawnow
end
plot(target(:,1),target(:,2),'x','MarkerSize', 20);
 %stops timer
%end
resultsTime = toc
%calculated how far away your robot is from the target.
resultsDis =  distance(target, botSim.getBotPos())

%test comment