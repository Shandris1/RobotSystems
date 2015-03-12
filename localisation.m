function [mean_particle, mean_pos, mean_ang, particles, par_pos, par_ang] = localisation(botSim,readings,num,particles,chosen_particles,mean_particle,max_dim,path_plan,par_pos,par_ang)

converged = 0; %The filter has not converged yet
n = 0;
draw = 1;
dist = zeros(1,num);
weight = zeros(1,num);
sigma = 60;
sigma_sense = 5;

%damping = 1/(readings*5);%this is amazing for sensor noise but takes too long
damping = 1/(readings*7);
converged_candidate = 0;
%countin_reinital = 0;
cloudsize = 0.5; %radius of the respawn cloud about previous particles
cloudangle = 4/180*pi; %angle of respawned particles
closest = 7*readings;%5cm per reading
botScan = botSim.ultraScan(); %get a scan from the real robot.
wall_follow_dist = 15;
search_long =1;
flag_random = 0;

while(converged == 0) %%particle filter loop
      
   n = n + 1;
   %% Write code to decide how to move next
       flag_move = 0;
       flag_turn = 0;
       max_found = 0;
       
       if(path_plan == 0)
            %find furthest wall
            if(search_long)
               if(n>1)
                   [max_val, max_ang_ind] = max(botScan);
               if( max_ang_ind == (readings/2 +1))
                   temp = botScan;
                   temp(max_ang_ind) =0;
                   [max_val, max_ang_ind] = max(temp);
               end
                else
                [max_val, max_ang_ind] = max(botScan);
                end
                turn = 2*pi/readings*(max_ang_ind-1)+rand(1)*pi/9;
                botSim.turn(turn);
                search_long = 0;
                move = 5;
                max_found = 1;
                flag_turn = 1;
            end

            if(botScan(1) > wall_follow_dist)||(max_found)
                move = 5;
                botSim.move(move);
                search_long = 0;
                max_found = 0;
                flag_move = 1;
            else
                search_long = 1;%why it doesnt move
            end
       else
           move = 5;
           botSim.move(move);
           flag_move = 1;
       end
       
       for i =1:num %for all the particles. 
            if (flag_turn == 1)
                particles(i).turn(turn);
            end
            if (flag_move == 1) 
                particles(i).move(move); %move the particle in the same way as the real robot
            end

            par_pos(i,:) =  particles(i).getBotPos;
            par_ang(i) = particles(i).getBotAng;

            par_ang(i) =  mod((par_ang(i)+2*pi),2*pi);
       end
         
    
    sigmax = std(par_pos(:,1))
    sigmay = std(par_pos(:,2))
    sigmaang  = std(par_ang)
    
    botScan = botSim.ultraScan(); %get a scan from the real robot.
    
    botScan(1) = 255;
    botScan(8) = -1;
    botScan(11) = -1;
    % introduce the vector which is supposed to contain the bad readings
    % from the real robot
 
    % iterate through the robot readings and find the bad values
    badReadings = find((botScan < 0) | (botScan > 254));
    
    readings = readings - length(badReadings);

    %% Write code for updating your particles scans
    counting = 0;
    curr_flag = 0;
    
    for i=1:num  
        
        
        
        %%check if particle outside the map
        if (particles(i).insideMap == 0) 
            
            if(min(botScan)>3)
            particles(i).randomPose(1);

            rand_pos = particles(i).getBotPos;
            checkx = find(round(par_pos(:,1)) == round(rand_pos(:,1)));
            if (~isempty(checkx))
                checky = find(round(par_pos(checkx,2)) == round(rand_pos(:,2)));
                if (~isempty(checky))
                    particles(i).randomPose(1);
                end
            end
            else
                particles(i).setBotPos([mean(par_pos(:,1)) mean(par_pos(:,2))] );
                chosen_particles(m).setBotAng(-mean(par_ang));
            end
                
        end
        if (sigma > 10*readings)&&(n>3)&&(flag_random == 0)
            particles(i).randomPose(1);

            rand_pos = particles(i).getBotPos;
            checkx = find(round(par_pos(:,1)) == round(rand_pos(:,1)));
            if (~isempty(checkx))
                checky = find(round(par_pos(checkx,2)) == round(rand_pos(:,2)));
                if (~isempty(checky))
                    particles(i).randomPose(1);
                end
            end
            curr_flag = 1;
        end
 
        parScan = particles(i).ultraScan();
        
        %remove the bad values from the particle readings vector
        parScan(badReadings) = [];
        
        for j =0 :readings-1
            check_orient(j+1) = sum((abs(circshift(parScan,j) - botScan)));
        end
        [lowest, best] = min((check_orient));
        %dist(i) = sqrt(sum((botScan - circshift(parScan,best - 1)) .^ 2)); 
        if(lowest < closest)
            particles(i).turn(-2*pi/readings * (best-1));      
            parScan = circshift(parScan,best-1);
            counting = counting +1;
            circ_ind(counting) = i;
        end

        error = normpdf((botScan - parScan),0,sigma_sense);
        error_sort = sort(error);
        %weight(i) = median(error_sort(1:0.75*length(error),1)) + damping;
        weight(i) = mean(error_sort(1:end*2/8)) + damping;
        dist(i)= sum((abs(parScan - botScan)));
      
    end
    
    if(curr_flag == 1)
        flag_random = 4;
    elseif(flag_random > 0) && (flag_random <= 4)
        flag_random = flag_random - 1;
    end

    flag_random
    n
    
    if(draw)   
         if botSim.debug()
            hold off; %the drawMap() function will clear the drawing when hold is off
            botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
            botSim.drawBot(30,'g'); %draw robot with line length 30 and green
            for i =1:num
                if(particles(i).insideMap == 1)
                    particles(i).drawBot(3); %draw particle with line length 3 and default color
                end
            end
%              for i =1:counting
%                 if(particles(i).insideMap == 1)
%                     particles(circ_ind(i)).drawBot(5,'r'); %draw particle with line length 3 and default color
%                 end
%             end
            drawnow;
         end
    end
    
    [sigma] = min(dist) 
    
    %%normalize weights
    weights = weight./sum(weight);
    
    %% Write code to check for convergence  
    if(6*sigmax < 0.15*max_dim)&&(6*sigmay < 0.15*max_dim)
        
        if botSim.debug()
            bot_pos = botSim.getBotPos
            bot_ang = botSim.getBotAng
        end
        %mean_pos = mean(par_pos);
        sort_weights = sort([weights' [1:num]']);
        index_average = sort_weights(end/4:end,2);
        mean_pos = mean(par_pos(index_average,:));
        %get weights sort them get IQR and then perform mean on those
        %indices
        sort_ang = sort(par_ang(index_average));
        range_ang = sort_ang(end) - sort_ang(1);
        if (range_ang>pi)
           index_to_change =  find((sort_ang>3/2*pi).*(sort_ang)>0);
           sort_ang(index_to_change) = sort_ang(index_to_change)-2*pi;
        end
        mean_ang = mean(sort_ang);
        %no_outlier =sort(par_ang);
       % mean_ang = mean(no_outlier(round(0.4*length(no_outlier)):round(0.6*length(no_outlier))));
        mean_particle.setBotPos(mean_pos);
        mean_particle.setBotAng(mean_ang);
        measurement = mean_particle.ultraScan();
        error_conv = sum((abs(measurement - botScan))./botScan)
        if(sum((abs(measurement - botScan)))< 3*readings)
%              if converged_candidate == 1
%                  converged = 1;
%                  n
%              else
%                  converged_candidate = 1;
%                  count_conv = count_conv  + 1
%              end
               converged = 1;          
         end
  
    else
        converged_candidate = 0;
    end
    
    %% Write code for resampling your particles
    c = weights(1);
    r = rand(1) / num;
    i = 1;
    for m=1:num
        U = r + (m-1) * (1 / num);
        while (U > c)
            i = i + 1;
            c = c + weights(i);
        end

        chosen_particles(m).setBotPos(particles(i).getBotPos());
        chosen_particles(m).setBotAng(particles(i).getBotAng());
    end

    for i=1:num
        particles(i).setBotPos(chosen_particles(i).getBotPos + [normrnd(0,cloudsize) normrnd(0,cloudsize)]);
        particles(i).setBotAng(chosen_particles(i).getBotAng + normrnd(0,cloudangle));
    end
end