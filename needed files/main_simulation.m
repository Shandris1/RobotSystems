clf;        %clears figures
clc;        %clears console
close all
clear;      %clears workspace
clear all
%axis equal; %keeps the x and y scale the same
% 
 %map=[0,0; 75,0; 150,0;150,30;30,30;30,150;120,150;120,120;60,120;60,60;90,60;90,90;150,90;150,180;0,180];
 %map = [0,20;20,0;60,0;80,20;80,60;60,80;20,80;0,60];
 %map = [35,0;70,35;120,35;155,0;155,25;180,25;145,60;145,110;180,145;155,145;155,180;120,135;70,125;35,180;35,145;0,145;45,110;45,60;0,25;35,25];

%map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];  %default map
%map = [-30,0;-30,40;30,40;30,60;5,60;45,90;85,60;60,60;60,40;120,40;120,60;95,60;135,90;175,60;150,60;150,40;210,40;210,60;185,60;225,90;265,60;240,60;240,40;300,40;300,0];
%map=[0,0;60,0;60,50;100,50;70,0;110,0;150,80;30,80;30,40;0,80]; %long map
%map = [0,0;60,0;60,45;45,45];
%map = [-30,0;-30,40;30,40;5,60;45,90;85,60;60,40;120,40;95,60;135,90;175,60;150,40;210,40;185,60;225,90;265,60;240,40;300,40;300,0];
%map = [-30,0;-30,40;30,40;30,45;5,45;5,85;85,85;85,45;60,45;60,40;120,40;120,45;95,45;135,65;175,45;150,45;150,40;210,40;210,45;185,45;225,65;265,45;240,45;240,40;300,40;300,0];
map = [-30,0;-30,40;30,40;30,45;5,45;45,65;85,45;60,45;60,40;120,40;120,45;95,45;135,65;175,45;150,45;150,40;210,40;210,45;185,45;225,65;500,45;240,45;240,40;300,40;300,0; 60,0; 60,-45;85,-45;45,-70;0,-45;30,-45;30,0;];

%map = [0,0; -30,-30; -25,-25; 30,-25; 30, 0;100,0;100,45;45,45];




botSim = BotSim(map);
botSim.setBotPos([20 28]);
botSim.setBotPos([60 84]);
%botSim.setBotAng(-pi/4);
botSim.setBotAng(-pi/6);
botSim.randomPose(10); %puts the robot in a random position at least 10cm away from a wall

target = round(botSim.getRndPtInMap(10));
%botSim.setBotPos([27.57 15.94]);
botSim.setBotPos([10 5]);
currentPos = (botSim.getBotPos());
currentPos = [10 5];
currentAng = (botSim.getBotAng());
%currentPos = round(currentPos);
currentAng = (currentAng);
figure
botSim.drawMap();
botSim.drawBot(300);
% h1 = plot(currentPos(1),currentPos(2), 'bo', 'YDataSource','currentPos(2)','XDataSource','currentPos(1)');
% x =[currentPos(1) currentPos(1)+cos(currentAng)*3];
% y = [currentPos(2) currentPos(2)+sin(currentAng)*3];
% h2 = plot(x,y,'b','YDataSource','y','XDataSource','x');
% hold on
target = [20.7 ;5.2]';
%target = [24.91;21.84];
%target = round(target);
dist = 8;
tic
[reduced, flag_cw]= reduce_graph(map,dist);
ending_red = toc
tic
out = visbil_full(reduced,botSim,target, flag_cw);
ending_vis_map = toc
tic
adjacency =visbil_point_reduced(reduced,target,out,dist,map);
ending_vis_target = toc
%adjacency = visbil_point(map,target,out)
%draw_adjacency(adjacency,reduced)
rob_size = 10;
stepsize = 1;%1cm as we need to be within 5 cm

%close_array = map_obstacle(map,1,1);% to be fixed when it comes to slanting lines
%[optimal dummy] = Astar(currentPos,currentAng,target,close_array,stepsize,botSim);
%[move turn optimal] = Astar(currentPos,currentAng,target,close_array,stepsize,botSim);
tic
[move turn optimal] = Astar_visbil(currentPos,currentAng, adjacency,reduced,botSim,3, target, dist,map);
ending_vastar_current_vis = toc
%vertex = turning_points(turn, optimal(end:-1:1,:))
for i=1:size(move)
delta =turn(i)- currentAng;
botSim.turn(delta);
botSim.move(move(i));
currentAng = turn(i);
currentPos = botSim.getBotPos();
%x =[currentPos(1) currentPos(1)+cos(currentAng)*3];
%y = [currentPos(2) currentPos(2)+sin(currentAng)*3];
botSim.drawBot(300);
%refreshdata(h1,'caller')
%refreshdata(h2,'caller')
drawnow
end
if(~isempty(optimal))
  %figure
%plot(optimal(:,1),optimal(:,2),'bo')
end


%localise_new(botSim,map,target)