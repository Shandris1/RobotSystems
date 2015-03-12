function [botSim] = localise(botSim,map,target)
%LOCALISE Template localisation function

%you can modify the map to take account of your robots configuration space
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);


botSim.drawMap();
rayno = 6;
botSim.setScanConfig(botSim.generateScanConfig(rayno))
[distance crossingPoint]  = botSim.ultraScan();
botSim.drawScanConfig();  %you should be aware that drawing and printing values will slow down your simulation during marking
botSim.drawBot(3);

%generate some random particles inside the map
random_amount = 10;
num =100;
particles(num,1) = BotSim;
for i = 1:num
    particles(i) = BotSim(modifiedMap);
    particles(i).randomPose(10);
    particles(i).setScanConfig(botSim.generateScanConfig(rayno))
end

% particle_expected_sdis = zeros(num,1);
particle_scores = zeros(num,1);
particle_normalised_scores = zeros(num,1);
particle_cumulative_normalised_scores = zeros(num,1);

particle_copy_poss = zeros(num,2);
particle_copy_angs = zeros(num,1);
for n = 1 : 10
    current_sdis = botSim.ultraScan();
    particle_scores_total = 0;
    for i =1:num
        particle_scores(i) = 0;          
        
        particle_scores(i) = sum(abs(current_sdis-particles(i).ultraScan()));
        
%      -sigmf(x,[0.1 10]);
        
        particle_scores(i) = 1/(particle_scores(i)+0.1);
        particle_scores_total = particle_scores_total+ particle_scores(i);
    end
    
    for i =1:num
        particle_normalised_scores(i) = particle_scores(i)/particle_scores_total;
        if(i ~= 1)
            particle_cumulative_normalised_scores(i) =  particle_cumulative_normalised_scores(i-1)+particle_normalised_scores(i);
        else
            particle_cumulative_normalised_scores(i) = particle_normalised_scores(i);
        end
    end
    
    roullete_selection_val = rand(1);
    jump_val = 1/num;
    for i =1:num
        for j =1:num
            if particle_cumulative_normalised_scores(j) > roullete_selection_val
                copy_particle_scores(i) = particle_scores(j);
                particle_copy_poss(i,:) = particles(j).getBotPos();
                particle_copy_angs(i) = particles(j).getBotAng();
                break
            end
        end
        roullete_selection_val = roullete_selection_val + jump_val;
        if roullete_selection_val > 1
            roullete_selection_val = roullete_selection_val-1;
        end
    end
    
    for i =1:num
        particles(i).setBotPos(particle_copy_poss(i,:) + 5*(rand(2,1)-[0.5;0.5])');
        particles(i).setBotAng(particle_copy_angs(i)+(rand(1)-0.5)*0.5);
    end
       
    for i =1:random_amount
        particles(i).randomPose(0);
    end
    
    turn = 0.05;
    move = 0.05;
    
    botSim.turn(turn);
    botSim.move(move);
    for i =1:num
        particles(i).turn(turn);
        particles(i).move(move);
    end
    
    if botSim.debug()
        hold off;       
        botSim.drawMap();
        axis manual;
        for i =1:num
            particles(i).drawBot(3);
        end
        botSim.drawBot(10,'g');
%         scatter(cps(:,1),cps(:,2),'marker','o','lineWidth',3);
        drawnow;
    end
end

%A simple but terrible way of localising
%Choose a random particle as the correct position
estimatedBotPos = particles(randi(length(particles)));
pos = estimatedBotPos.getBotPos();
ang = estimatedBotPos.getBotAng();

%Input estimated pose into pathplanning.  Here the pathplanning function is
%just an example and needs to be completed.
[movements turns] = pathPlan(pos,ang,target,modifiedMap);

%Turn and move the supplied botSim object.  The instruction you give to the robot
%will be recorded and analysed by the automarker
for i = 1:length(movements)
    botSim.turn(turns(i));
    botSim.move(movements(i));
end
end

