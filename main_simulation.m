clf;        %clears figures
clc;        %clears console
close all
clear;      %clears workspace
clear all
%axis equal; %keeps the x and y scale the same
%map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];  %default map
map = [-30,0;-30,40;30,40;30,60;5,60;45,90;85,60;60,60;60,40;120,40;120,60;95,60;135,90;175,60;150,60;150,40;210,40;210,60;185,60;225,90;265,60;240,60;240,40;300,40;300,0];
%map=[0,0;60,0;60,50;100,50;70,0;110,0;150,80;30,80;30,40;0,80]; %long map

botSim = BotSim(map);
botSim.setBotPos([20 28]);
%botSim.setBotAng(-pi/4);
botSim.setBotAng(-pi/6);
botSim.randomPose(10); %puts the robot in a random position at least 10cm away from a wall
target = round(botSim.getRndPtInMap(10));
currentPos = (botSim.getBotPos());
currentAng = (botSim.getBotAng());
currentPos = round(currentPos);
currentAng = (currentAng);
%botSim.drawMap();
%botSim.drawBot(3);
% h1 = plot(currentPos(1),currentPos(2), 'bo', 'YDataSource','currentPos(2)','XDataSource','currentPos(1)');
% x =[currentPos(1) currentPos(1)+cos(currentAng)*3];
% y = [currentPos(2) currentPos(2)+sin(currentAng)*3];
% h2 = plot(x,y,'b','YDataSource','y','XDataSource','x');
% hold on
%target = [45;84];
target = round(target);
rob_size = 10;
stepsize = 1;%1cm as we need to be within 5 cm
tic;
close_array = map_obstacle(map,1,1);% to be fixed when it comes to slanting lines
%[optimal dummy] = Astar(currentPos,currentAng,target,close_array,stepsize,botSim);
[move turn optimal] = Astar(currentPos,currentAng,target,close_array,stepsize,botSim);
% for i=1:size(move)
% delta =turn(i)- currentAng;
% botSim.turn(delta);
% botSim.move(move(i));
% currentAng = turn(i);
% currentPos = botSim.getBotPos();
% %x =[currentPos(1) currentPos(1)+cos(currentAng)*3];
% %y = [currentPos(2) currentPos(2)+sin(currentAng)*3];
% botSim.drawBot(3);
% %refreshdata(h1,'caller')
% %refreshdata(h2,'caller')
% drawnow
% end
if(~isempty(optimal))
  %figure
%plot(optimal(:,1),optimal(:,2),'bo')
end
ending = toc;

%localise_new(botSim,map,target)