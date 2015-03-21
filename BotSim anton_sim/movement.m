function[mean_particle] = movement(num,particles,mean_particle,moveCmd,turnCmd,optimal,speed)

exit = 0;

for i=1:length(moveCmd)
    par_pos = [];
    par_ang = [];
    
    turn_cmd = turnCmd(i);   
    move = moveCmd(i);
    turn = turn_cmd - mean_particle.getBotAng();
        
    if(turn > pi/4)
        exit = 1;
        break;
    end
    
    if(exit == 0) 
        turning(speed,turn);
        driving(speed,move);

        for j=1:num
            particles(j).turn(turn);
            particles(j).move(move);

            par_pos(j,:) =  particles(j).getBotPos;
            par_ang(j) = particles(j).getBotAng;
        end

        mean_pos = mean(par_pos);
        mean_ang = mean(par_ang);
        mean_particle.setBotPos(mean_pos);
        mean_particle.setBotAng(mean_ang);
    end
    
    if botSim.debug()
        hold off; %the drawMap() function will clear the drawing when hold is off
        botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
        plot(target(:,1),target(:,2),'x','MarkerSize', 40);
        botSim.drawBot(30,'r'); %draw robot with line length 30 and green
        for i =1:num
            %if(particles(i).insideMap == 1)
                particles(i).drawBot(3); %draw particle with line length 3 and default color
           % end
        end
         for i =1:counting
            if(particles(i).insideMap == 1)
                particles(circ_ind(i)).drawBot(15,'r'); %draw particle with line length 3 and default color
            end
        end
        drawnow;
    end
    
end           