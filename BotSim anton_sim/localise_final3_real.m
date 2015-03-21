function [mean_particle, particles, mean_pos mean_ang] = localise_final3_real(botSim,mean_particle,chosen_particles,particles,map,target,readings, noiselevel,num,max_dim_vec,path_plan,speed)
%If readings are not divisible by 4, the wall-following program will crash

draw = 1
angle = [];
mot = NXTMotor('B');
scanSpeed = 30;
samples = 16;
OpenUltrasonic(SENSOR_4);
max_dim = max_dim_vec(1);
%% Localisation code
saturation_value = sqrt(max_dim_vec(2)^2+max_dim_vec(3)^2);
maxNumOfIterations = 500;
n = 0;
converged =0; %The filter has not converged yet
dist = zeros(1,num);
weight = zeros(1,num);
sigma = 60;

%robot param
sigma_sense = (10 + noiselevel/2);
if(path_plan)
    sigma_sense = (2 + noiselevel/4);
end

closest = 9*readings;%8cm per reading
cloudsize = 0.5 + noiselevel*0.1; %radius of the respawn cloud about previous particles
cloudangle = 4/180*pi; %angle of respawned particles
wall_follow_dist = 15 + noiselevel*0.8;

blobcheck= 0;
damping = 0;
converged_candidate = 0;
countin_reinital = 0;
search_long = 1;
flag_random = 0;
reinit_count = 0;
found_longest = 0;

[botScan angle] = ultraScan(scanSpeed,readings) %get a scan from the real robot.
% iterate through the robot readings and find the bad values
badReadings = find((botScan < 0) | (botScan > 254)| (botScan > saturation_value));
botScan_move = botScan;
readings_update = readings - length(badReadings);
botScan(badReadings) = [];
%satur_ind = find(botScan>saturation_value);
%botScan(satur_ind) = saturation_value;
%satur_ind =[];
if (~isempty(badReadings))
    if(badReadings(1) == 1)
        botScan_move(1) = min(botScan_move([2 readings]));
        botScan_move(1) = botScan_move(1).*(botScan_move(1)<254).*(botScan < saturation_value);
%         if(botScan_move(2) > 254) || (botScan_move(2) < 0) || (botScan_move(readings) > 254) || (botScan_move(readings) < 0)
%             botScan_move(1) = botScan_move(1).*(botScan_move(1)<254);
%         else             
%             botScan_move(1) = min(botScan([2 readings_update]));
%         end
    end
end
%satur_ind = find((botScan_move>saturation_value) & (botScan_move<255)& (botScan_move>0));
%botScan_move(satur_ind) = saturation_value;
%satur_ind =[];
while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    
    n = n+1; %increment the current number of iterations
    %% Write code to decide how to move next
   turn_long = 0;
   wall_avoidance = 0;
   flag = 0;
   flag_long_turn = 0;
   flag_long_move = 0;
   badReadings = [];
   % pause(1);
%     if (botScan(1) < 10)
%         wall =1;
%         flag = 1;
%         botSim.turn(turn);
%     elseif (wall == 1) && (botScan(8) > 20)
%         botSim.turn(-turn);
%         flag = 2;
%     end
%     if (botScan(1) < 10)
%         botSim.turn(pi/2);
%     elseif (botScan(3) < 10 && botScan(1) > 10)
%         botSim.turn(-pi/2);
%     end
%     if ((sum(botScan(2:readings)) < wall_follow_dist) == 0) && (botScan(1) > wall_follow_dist*2)
%          move = 10;
%          botSim.move(move);
%          flag = 1;   
%     elseif ((sum(botScan(2:readings)) < wall_follow_dist) == 0) && (botScan(1) > wall_follow_dist)
%          move = 3;
%          botSim.move(move);
%          flag = 2;
%     elseif (botScan(1) < wall_follow_dist)
%          turn = pi/2;
%          botSim.turn(turn);
%          flag = 3;
%     elseif (botScan((readings/4)+1) <= wall_follow_dist)
%          move = 3;
%          botSim.move(move);
%     end

        leftReading = readings/4+1;
        rightReading = 3*readings/4+1;

        goodLeftReadings = find((botScan_move(1:leftReading) > 0) | (botScan_move(1:leftReading) < 255));
        goodRightReadings = find((botScan_move(rightReading:readings) > 0) | (botScan_move(rightReading:readings) < 255)) + (readings - (readings/4));

        left = (botScan_move(goodLeftReadings) < 20);
        right = (botScan_move(goodRightReadings) < 20);
        sum_reflex = sum(botScan_move(goodLeftReadings)) + sum(botScan_move(goodRightReadings));

        if (sum(left) >= 1) || (sum(right) >=1) 
            move_neg = -7;
            driving(speed,move_neg);
            
            if (sum(left) >= 1) && (sum(right) >= 1)
                
                [max_val max_ang_ind] = max(botScan_move(leftReading:rightReading).*(botScan_move(leftReading:rightReading)<255).*(botScan_move(leftReading:rightReading)>0));
                max_ang_ind=max_ang_ind+(readings/4);
                 
                turn = 2*pi/readings*(max_ang_ind-1)+randn(1)*pi/18; 
                move_pos = 3;
            elseif(sum(left) >= 1)
                turn = -(pi/3)+randn(1)*pi/18;
                move_pos = 7;
            elseif(sum(right) >= 1)
                turn = pi/3+randn(1)*pi/18;
                move_pos = 7;
            end

            turning(speed, turn);
            driving(speed,move_pos);
            
            flag = 1;
            wall_avoidance = 1;
            found_longest = 0;
            search_long = 1;
        
        elseif (sum(left) < 1) && (sum(right) < 1)
            if((search_long == 0) && (min([botScan_move(1) botScan_move(2) botScan_move(readings)]) > wall_follow_dist)) || (found_longest) 
                move = 7;
                driving(speed,move);
                search_long = 0;
                flag = 2;
            elseif (wall_avoidance == 1)
                search_long = 1;
            end
            wall_avoidance = 0;
%        elseif (sum_reflex > 550)
%             move_neg = -7;
%             driving(speed,move_neg);
% 
%             turn = -(pi/3)+randn(1)*pi/36;
%             turning(speed, turn);
% 
%             move_pos = 7;
%             driving(speed,move_pos);
% 
%             flag = 1;
%             wall_avoidance = 1;
%             search_long = 1;
%             found_longest = 0;
        end
       %find furthest wall
        if(wall_avoidance == 0) && (search_long) && (found_longest == 0)
%             if(n>1)
%                 [max_val, max_ang_ind] = max(botScan_move.*(botScan_move<254));
%                 if (max_ang_ind == (readings/2 +1))
%                    %anything above 255 set to 0 
%                    temp = botScan_move.*(botScan_move<254);
%                    temp(max_ang_ind) =0;
%                    [max_val, max_ang_ind] = max(temp);
%                 end
%             else
%                 [max_val, max_ang_ind] = max(botScan_move.*(botScan_move<254));
%             end
% 
%             %turn = 2*pi/readings_update*(max_ang_ind-1)+rand(1)*pi/9;
%             temp_vec = botScan_move.*(botScan_move<254);
%             if(max_ang_ind == 1)
%                 turn_long = 2*pi/readings*((-1*temp_vec(mod(max_ang_ind -1-1 + readings,readings)+1) + mod(max_ang_ind-1 + readings,readings)*temp_vec(max_ang_ind) +mod(max_ang_ind + readings,readings)*temp_vec(mod(max_ang_ind + 1-1 + readings,readings)+1) )/(sum(temp_vec([mod(max_ang_ind -1-1 + readings,readings)+1 mod(max_ang_ind -1 + readings,readings)+1 mod(max_ang_ind - 1+ 1+ readings,readings)+1]))))+randn(1)*pi/18;
%             elseif(max_ang_ind == readings)
%                  turn_long = 2*pi/readings*((mod(max_ang_ind -2 + readings,readings)*temp_vec(mod(max_ang_ind -1-1 + readings,readings)+1) + mod(max_ang_ind-1 + readings,readings)*temp_vec(max_ang_ind) +readings*temp_vec(mod(max_ang_ind + 1-1 + readings,readings)+1) )/(sum(temp_vec([mod(max_ang_ind -1-1 + readings,readings)+1 mod(max_ang_ind -1 + readings,readings)+1 mod(max_ang_ind - 1+ 1+ readings,readings)+1]))))+randn(1)*pi/18;
%             else
%             turn_long = 2*pi/readings*((mod(max_ang_ind -2 + readings,readings)*temp_vec(mod(max_ang_ind -1-1 + readings,readings)+1) + mod(max_ang_ind-1 + readings,readings)*temp_vec(max_ang_ind) +mod(max_ang_ind + readings,readings)*temp_vec(mod(max_ang_ind + 1-1 + readings,readings)+1) )/(sum(temp_vec([mod(max_ang_ind -1-1 + readings,readings)+1 mod(max_ang_ind -1 + readings,readings)+1 mod(max_ang_ind - 1+ 1+ readings,readings)+1]))))+randn(1)*pi/18;
%             %turn_long = 2*pi/readings*(max_ang_ind-1)+rand(1)*pi/18;

           [sort_botscan sort_botscan_ind] = sort(botScan_move,'descend');

            found = 0;
            count = 1;
            while (found == 0 ) 
                top_value = sort_botscan(1);
                top_ind = sort_botscan_ind(1);
                top_ind_prev = top_ind-1;
                top_ind_next = top_ind+1;
                
                if (top_ind_prev == 0)
                    top_ind_prev = readings;
                elseif (top_ind_next == readings+1)
                    top_ind_next = 1;
                end
                
                if(botScan_move(top_ind)*0.5 <= botScan_move(top_ind_prev) && botScan_move(top_ind)*0.5 <= botScan_move(top_ind_next))
                    found = 1;
                    move = 7;
                    turn_long = 2*pi/readings*(top_ind-1)+rand(1)*pi/18;
                    
                    turning(speed,turn_long);
                    driving(speed,move);
                    
                    search_long = 0;
                    flag_long_turn = 1;
                    flag_long_move = 1;
                    found_longest = 1;
                else
                    sort_botscan(1) = [];
                    sort_botscan_ind(1) = [];
                   
                end
                           
            end
            if(found == 0)
               [max_val, max_ang_ind] = max(botScan_move.*(botScan_move<254));
               turn_long = 2*pi/readings*(max_ang_ind-1)+rand(1)*pi/18;
               
               turning(speed,turn_long);
               driving(speed,move);
               
               search_long = 0;
               flag_long_turn = 1;
               flag_long_move = 1;
               found_longest = 1;
            end
        end
%        elseif (path_plan)&&(localise == 0)
%            turn = pi/36 + randn(1)*pi/36;
%            turning(speed,turn);
%            flag = 3;
%        end
        
%     if (botScan(1) < 10) || (botScan((readings/4)+1) < wall_follow_dist-5)
%         turn = pi/30;
%         botSim.turn(turn);
%         flag = 1;
%     elseif ((botScan((readings/4)+1)) > wall_follow_dist+5)
%         turn = pi/30;
%         botSim.turn(-turn);
%         flag = 2;
%     else
%         botSim.move(move);
%         flag = 3;
%     end


    
%     if (converged_candidate == 1)&&(flag==0)
%         flag = 5;
%       %  turn = pi/6;
%        % botSim.turn(turn);  %%convergence test turn
%     end
 
    %pause (1);
    for i =1:num %for all the particles. 
        %%particles(i).turn(particles(i).getBotAng); %turn the particle in the same way as the real robot
        
        if(flag == 1)
            particles(i).move(move_neg);
            particles(i).turn(turn);
            particles(i).move(move_pos);
        elseif (flag == 2) 
            particles(i).move(move);
%         elseif (flag == 3)
%             particles(i).turn(turn);
%         elseif (flag == 4)
%             particles(i).turn(turn);
%             particles(i).move(move);
        end
        
        if(flag_long_turn == 1)
            particles(i).turn(turn_long)
        end
        
        if(flag_long_move == 1)
            particles(i).move(move);
        end
        
        par_pos(i,:) =  particles(i).getBotPos;
        par_ang(i) = particles(i).getBotAng;
        
        
        %to code in one line using visbility code!
%         if (par_ang(i)>0)
%             par_ang(i) =  mod(par_ang(i),2*pi);
%         else
%             par_ang(i) =  mod(par_ang(i),-2*pi);
%             par_ang(i) =  par_ang(i) + 2*pi;
%         end
         par_ang(i) =    mod((par_ang(i)+2*pi),2*pi);
    end
    
    sigmax = std(par_pos(:,1));
    sigmay = std(par_pos(:,2));
    sigmaang  = std(par_ang);
    
    [botScan angle] = ultraScan(scanSpeed,readings) %get a scan from the real robot.
    
    % iterate through the robot readings and find the bad values
    badReadings = find((botScan < 0) | (botScan > 254)| (botScan > saturation_value));
    readings_update = readings - length(badReadings);
    botScan_move = botScan;
    botScan(badReadings) = [];
%     satur_ind = find(botScan>saturation_value);
%     if(~isempty(satur_ind))
%         stop = 0;
%     end
   % botScan(satur_ind) = saturation_value;
    %satur_ind =[];
    if(~isempty(badReadings))
         if(badReadings(1) == 1)
         botScan_move(1) = min(botScan_move([2 readings]));
         botScan_move(1) = botScan_move(1).*(botScan_move(1)<254).* (botScan < saturation_value);
%         if(botScan_move(2) > 254) || (botScan_move(2) < 0) || (botScan_move(readings) > 254) || (botScan_move(readings) < 0)
%             botScan_move(1) = botScan_move(1).*(botScan_move(1)<254);
%         else             
%             botScan_move(1) = min(botScan([2 readings_update]));
%         end
        end
    end
    
%     satur_ind = find((botScan_move>saturation_value) & (botScan_move<255)& (botScan_move>0));
%     botScan_move(satur_ind) = saturation_value;
    
    %min_bot_scan = min(botScan);
    %% Write code for updating your particles scans
    %%check measurements using p(z|r) in ideal case std is v small in reality
    %%we can add sensor noise
    counting = 0;
    curr_flag = 0;
    for i=1:num  
        %%check if particle outside the map
        if (particles(i).insideMap == 0) 
            
            if(min(botScan)>3+noiselevel/2)
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
                particles(i).setBotAng(-mean(par_ang));
            end
                
        end
        if ((sigma > (noiselevel*1.6+ 5)*readings_update)&&(n>3)&&(flag_random == 0))
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
        
        for j =0 :readings_update-1
            check_orient(j+1) = sum((abs(circshift(parScan,j) - botScan)));
        end
        [lowest, best] = min((check_orient));
        %dist(i) = sqrt(sum((botScan - circshift(parScan,best - 1)) .^ 2)); 
        if(lowest < closest)
            particles(i).turn(-2*pi/readings_update * (best-1));      
            parScan = circshift(parScan,best-1);
            counting = counting +1;
            circ_ind(counting) = i;
        end

        %%%weight determination
        % using worst readings
        %error = normpdf((botScan - parScan),0,sigma_sense);
%         error = exp((-((botScan - parScan)).^2)/(2*(sigma_sense)^2));
%         error_sort = sort(error);
%          weight(i) = mean(error_sort(1:round(end*2/8)+1)) + damping;
%         
        
%         %using mean of sep gauss
%         
%         error = normpdf((botScan - parScan),0,sigma_sense);
%         weight(i) = mean(error) + damping;
%        
%         % using mean of iqr guass
%         
%         error = normpdf((botScan - parScan),0,sigma_sense);
%         error_sort = sort(error);
%         weight(i) = mean(error_sort(1:round(0.75*length(error)),1)) + damping;
%         
%         % using median of iqr guass
%         
%         error = normpdf((botScan - parScan),0,sigma_sense);
%         weight(i) = median(error) + damping;
%         
        % using sum of error and do guass
        
        error = sum(abs(botScan - parScan));
        weight(i) = exp((-(error)^2)/(2*(sigma_sense*readings_update*0.6)^2));
       
%         % using sum of error no gauss
%         error = sum(abs(botScan - parScan));
%         weight(i) = 1/(error + 1e-100000000000000);
%         
%         
        
%         
%         error = normpdf((botScan - parScan),0,sigma_sense);
%         error_sort = sort(error);
%         weight(i) = median(error_sort(1:0.75*length(error),1)) + damping;
        %weight(i) = mean(error_sort) + damping;
        dist(i)= sum((abs(parScan - botScan)));
      
    end
    
    reinit_count = 0;
    
    if(curr_flag == 1)
        flag_random = 4;
    elseif(flag_random > 0) && (flag_random <= 4)
        flag_random = flag_random - 1;
    end
    
%     if (flag_random_in == 1)
%         flag_random = 2;
%         flag_random_in =0;
%     elseif (flag_random == 2)
%         flag_random = 0;
%     end
    flag_random
    n
    
    if(draw)   
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
    
    
    [sigma] = min(dist) 
%     if(sigma>10) && (flag_random)
%        countin_reinital = countin_reinital + 1
%        
%        reinitialise_flag = 1;
%        flag_random = 0;
% 
%     end
    if((6*sigmax < 0.4*max_dim)&&(6*sigmay < 0.4*max_dim))||(path_plan)
        blobcheck = 1;
        sigma_sense = 2 + noiselevel/4;
    else
        blobcheck = 0;
        sigma_sense = 10 + noiselevel/2;
    end
    %% Write code for scoring your particles
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
        index_average = sort_weights(round(end/4):end,2);
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
        measurement(badReadings) = [];
        error_conv = sum((abs(measurement - botScan)))
        if(sum((abs(measurement - botScan)))< (3+noiselevel)*readings_update)
%                if(path_plan)
%                     localise = 1;
%                     mean_particle.drawBot(30,'g'); %draw robot with line length 30 and green
%                     [moveCmd turnCmd optimal] = Astar_visbil(mean_pos,mean_ang, adjacency,reduced,botSim,stepsize,target, dist,map);
% 
%                     if(sum(moveCmd) <= stepsize)
%                         driving(speed,moveCmd);
%                         converged = 1;
%                     end
%                else
                   converged = 1;
               %end
        end
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
end
