function [mean_particle, particles, mean_pos mean_ang , botScan] = real_robot_localisation(botSim,mean_particle,chosen_particles,particles,map,target,readings, noiselevel,num,path_plan,max_dim_vec,moveCmd,turnCmd,optimal,localise,stepsize,adjacency,reduced,path_dist, botScan)
%% Localisation code
%If readings are not divisible by 4, the wall-avoidance program will crash

%user changeable parameters
draw = 1;
speed = 70; %used for driving and turning
vector_indices = 1:readings; 
scanSpeed = 30; %used for sensor motor
sigma = 60; %used for lost functionality
maxNumOfIterations = 500;
readings_lost = 4;
sigma_sense = (4 + noiselevel/2);
if(path_plan)
    sigma_sense = (0.1 + noiselevel/4);
end
closest = 3.5;%8cm per reading
cloudsize = 0.5 + noiselevel*0.2; %radius of the respawn cloud about previous particles
cloudangle = 4/180*pi; %angle of respawned particles
wall_follow_dist = 15 + noiselevel*0.8;
damping = 0.01;
damping = 0.015;
damping = 0;
wall_threshold = 20;

max_dim = max_dim_vec(1); %max dimension of the map
saturation_value = sqrt(max_dim_vec(2)^2+max_dim_vec(3)^2); %maxium values of what sensors can read
dist = zeros(num,2);
weight = zeros(1,num);
par_pos= zeros(num,2);
par_ang= zeros(num,1);
%flags 
blobcheck = 0;
n = 0;
converged = 0; 
search_long = 1;
flag_random = 0;
found_longest = 0;
best_flag = 1.6;

if(path_plan == 0)
    [botScan angle] = ultraScan(scanSpeed,readings); %get a scan from the real robot.
    % iterate through the robot readings and find the bad values
end

%check for readings < 0 and > 254 and > saturation value
badReadings = find((botScan < 0) | (botScan > 254)| (botScan > saturation_value));

%set botScan_move including bad readings
botScan_move = botScan;
readings_update = readings - length(badReadings);
botScan(badReadings) = [];

%if first reading is a bad reading then calculate min between prev and next
%reading, if invalid set to 0
if (~isempty(badReadings))
    if(badReadings(1) == 1)
        botScan_move(1) = min(botScan_move([2 readings]));
        botScan_move(1) = botScan_move(1).*(botScan_move(1)<254).*(botScan(1) < saturation_value);
    end
end

while(converged == 0 && n < maxNumOfIterations)
    
    n = n+1; 
    
    %% Write code to decide how to move next
    
    %motion strategy flags
    turn_long = 0;
    wall_avoidance = 0;
    flag = 0;
    flag_long_turn = 0;
    flag_long_move = 0;
 
    badReadings = [];

    %left hemisphere readings
    leftReading = readings/4+1;
    %right hemisphere readings
    rightReading = 3*readings/4+1;

    %remove bad readings from left and right quarters
    goodLeftReadings = find((botScan_move(1:leftReading) > 0) | (botScan_move(1:leftReading) < 255) | (botScan_move(1:leftReading) < saturation_value));
    goodRightReadings = find((botScan_move(rightReading:readings) > 0) | (botScan_move(rightReading:readings) < 255) | (botScan_move(rightReading:readings) < saturation_value)) + (readings - (readings/4));

    %check if you're close to the wall
    if(path_plan == 0)
        left = (botScan_move(goodLeftReadings) < wall_threshold);
        right = (botScan_move(goodRightReadings) < wall_threshold);
    else
        left = (botScan_move(goodLeftReadings) < path_dist);
        right = (botScan_move(goodRightReadings) < path_dist);
    end

    %encountered a wall
    if (sum(left) >= 1) || (sum(right) >=1) 
        move_neg = -7;
        driving(speed,move_neg);

        %encountered a wall head on or a corner
        if (sum(left) >= 1) && (sum(right) >= 1)

            %finding max valid reading in third and fourth quadrant
            [max_val max_ang_ind] = max(botScan_move(leftReading:rightReading).*(botScan_move(leftReading:rightReading)<255).*(botScan_move(leftReading:rightReading)>0).*(botScan_move(leftReading:rightReading)<saturation_value));
            max_ang_ind=max_ang_ind+(readings/4);

            %turn towards furthest distance + random angle 
            turn = 2*pi/readings*(max_ang_ind-1)+ path_plan*randn(1)*pi/18; 
            move_pos = 3;
        %encountered a wall on the left
        elseif(sum(left) >= 1)
            turn = -(pi/3)+ path_plan*randn(1)*pi/18;
            move_pos = 7;
        %encountered a wall on the right
        elseif(sum(right) >= 1)
            turn = pi/3+ path_plan*randn(1)*pi/18;
            move_pos = 7;
        end
        
        if(path_plan == 0)
            %before path planning we round turn to 90 degrees to ensure
            %robot is perpendicular to walls
            turn = round(turn/pi*2)*pi/2;
        end
        turning(speed, turn);
        driving(speed,move_pos);

        flag = 1; %movements for particles
        wall_avoidance = 1; %wall encountered
        search_long = 1; %search for longest distance on the next iteration
        found_longest = 0; %longest distance is still not found
        localise = 0;
    %no wall encountered 
    elseif (path_plan == 0) && (sum(left) < 1) && (sum(right) < 1)
        %first, second and last readings are near a wall
        if(search_long == 0 && min([botScan_move(1) botScan_move(2) botScan_move(readings)]) > wall_follow_dist) || (found_longest)
            move = 7;
            driving(speed,move);
            search_long = 0;
            flag = 2;
        %if you just encountered a wall check for longest distance next
        %iteration
        elseif (wall_avoidance == 1)
            search_long = 1;
        end
        wall_avoidance = 0;
    end
   %find furthest wall before path planning
   if(path_plan == 0)
     if(wall_avoidance == 0) && (search_long) && (found_longest == 0)
         
        %sort botScan_move in descending order 
        [sort_botscan sort_botscan_ind] = sort(botScan_move,'descend');

        found = 0;
        while (found == 0) 
            %get top, prev and next values
            top_value = sort_botscan(1);
            top_ind = sort_botscan_ind(1);
            top_ind_prev = top_ind-1;
            top_ind_next = top_ind+1;

            %check for valid index
            if (top_ind_prev == 0)
                top_ind_prev = readings;
            elseif (top_ind_next == readings+1)
                top_ind_next = 1;
            end

            %check 50% of value in current index is less than previous and
            %less than next, set longest if found
            if(botScan_move(top_ind)*0.5 <= botScan_move(top_ind_prev) && botScan_move(top_ind)*0.5 <= botScan_move(top_ind_next))
                found = 1;
                move = 7;
                
                turn_long = 2*pi/readings*(top_ind-1);
                turn_long = round(turn_long/pi*2)*pi/2;
                
                turning(speed,turn_long);
                driving(speed,move);

                search_long = 0;
                
                %set particle turn and movements flags
                flag_long_turn = 1;
                flag_long_move = 1;
                found_longest = 1;
            else
                %remove first element of sorted vector
                sort_botscan(1) = [];
                sort_botscan_ind(1) = [];
            end

        end
        if(found == 0)
           %if condition above is never satisfied get max reading
           [max_val, max_ang_ind] = max(botScan_move.*(botScan_move<255).*(botScan_move < saturation_value));
           
           turn_long = 2*pi/readings*(max_ang_ind-1);
           turn_long = round(turn_long/pi*2)*pi/2;
           
           turning(speed,turn_long);
           driving(speed,move);

           search_long = 0;
           flag_long_turn = 1;
           flag_long_move = 1;
           found_longest = 1;
        end
     end
   %trigger movement in tracker mode
   elseif(path_plan) && (wall_avoidance == 0)
       %if not localised 
       if (localise == 0)
           turn = pi/36 + randn(1)*pi/36;
           turn = pi/6 + randn(1)*pi/36;
           %%check tomorrow perhaps decrease
           %turn = round(turn/pi*2)*pi/2;
           if (abs(turn) <=pi/36)
               turn = pi/36;
           end
          turning(speed,turn);
          flag = 3;
       else
           turn_cmd = turnCmd(1);
           turn = turn_cmd - mean_particle.getBotAng();
           turn = mod(turn+ 2*pi,2*pi);

           if  turn > pi
                turn = 2*pi -turn; %make robot take shortest possible turn 
           end

           turning(speed,turn);
           flag = 3;
           
           %check if turn is big/small
           if(abs(turn) < pi/18)
                move = moveCmd(1);
                driving(speed,move);
                flag = 4;
                localise = 1;
           else
                localise = 0;
           end
       end 
   end
      
   %particle movement
   for i =1:num 
        if(flag == 1)
            %move and turn when wall encountered
            particles(i).move(move_neg);
            particles(i).turn(turn);
            particles(i).move(move_pos);
        elseif (flag == 2) 
            particles(i).move(move);
        elseif (flag == 3)
            particles(i).turn(turn);
        elseif (flag == 4)
            particles(i).turn(turn);
            particles(i).move(move);
        end

        %flags used for longest distance
        if(flag_long_turn == 1)
            particles(i).turn(turn_long)
        end

        if(flag_long_move == 1)
            particles(i).move(move);
        end
        %TODO move after weights
        par_pos(i,:) =  particles(i).getBotPos;
        par_ang(i) = particles(i).getBotAng;
        par_ang(i) =    mod((par_ang(i)+2*pi),2*pi);
   end
    
    %computing statistics to be used in convergence 
    sigmax = std(par_pos(:,1));
    sigmay = std(par_pos(:,2));
   
    
    %perform ultrascan
    [botScan angle] = ultraScan(scanSpeed,readings) ;
    
    % iterate through the robot readings and find the bad values  
    badReadings = find((botScan < 0) | (botScan > 254)| (botScan > saturation_value));
    readings_update = readings - length(badReadings);
    botScan_move = botScan;
    
    if(~isempty(badReadings))
         if(badReadings(1) == 1)
             botScan_move(1) = min(botScan_move([2 readings]));
             botScan_move(1) = botScan_move(1).*(botScan_move(1)<254).* (botScan(1) < saturation_value);
         end
    end
    
    %% Write code for updating your particles scans
    if(localise ==0)&&(flag ~=4)
        counting = 0;
        curr_flag = 0;
        for i=1:num  
            %%check if particle outside the map
            if (particles(i).insideMap == 0) 
                %reintialize outside particles
                particles(i).randomPose(10);
                rand_pos = particles(i).getBotPos;

                %prevent particles reintializing in previous position 
                checkx = find(round(par_pos(:,1)) == round(rand_pos(:,1)));
                if (~isempty(checkx))
                    checky = find(round(par_pos(checkx,2)) == round(rand_pos(:,2)));
                    if (~isempty(checky))
                        particles(i).randomPose(10);
                    end
                end    
            end
            %check if robot gets lost - check sigma (best particle) greater
            %than current noiselevel * reduced readings (4 or 3)
            %apply after 3 iteratons and prevent getting lost consecutively
            if ((sigma > (noiselevel*best_flag+ 3)*readings_lost)&&(n>3)&&(flag_random == 0))
                particles(i).randomPose(10);

                rand_pos = particles(i).getBotPos;
                checkx = find(round(par_pos(:,1)) == round(rand_pos(:,1)));
                if (~isempty(checkx))
                    checky = find(round(par_pos(checkx,2)) == round(rand_pos(:,2)));
                    if (~isempty(checky))
                        particles(i).randomPose(10);
                    end
                end
                curr_flag = 1;
            end

            %check crosspoints of particle to locate perpendicular scan ray to
            %a wall
            [parScan, crosspoints] = particles(i).ultraScan();
            par_pos(i,:) =  particles(i).getBotPos();

            %sort difference of x particle position and x position of
            %crosspoints for half the readings
            [value index_to_check]  = sort(abs(crosspoints(1: readings/2,1)-par_pos(i,1)));

            %increment by readings/4 from the first perpendicular ray found 
            vector_indices = sort([mod(index_to_check(1)-1 + readings, readings)+1 mod(index_to_check(1)-1 + readings/4 + readings, readings)+1 mod(index_to_check(1)-1 + readings/2 +  readings, readings)+1 mod(index_to_check(1)-1 + + readings*3/4 + readings, readings)+1]);

            %get these vector indices from particle and bot scan
            par_sense = parScan(vector_indices);
            bot_sense = botScan(vector_indices);

            %check for bad readings
            bad_meas = find((bot_sense < 0) | (bot_sense > 254)| (bot_sense > saturation_value));
            readings_num = 4 - length(bad_meas);

            %remove the bad values from the particle, bot readings and vector_indices vector
            bot_sense(bad_meas) = []; 
            par_sense(bad_meas) = [];
            vector_indices(bad_meas) = [];

            %update sigma_sense if current bot got lost
            if(curr_flag)
                sigma_sense = 4 + noiselevel/2;
            end

            %compute weights 
            %distance between bot_sense and par sense 
            error = sum(abs(bot_sense - par_sense));
            weight(i) = exp((-(error)^2)/(2*(sigma_sense*readings_num*0.5)^2)) + damping;

            %calculate error distance to be used to compute best particle
            dist(i,:)= [error readings_num];
        end

        %reset lost flag and decrement up till 4 iterations 
        if(curr_flag == 1)
            flag_random = 4;
        elseif(flag_random > 0) && (flag_random <= 4)
            flag_random = flag_random - 1;
        end

        %Draw bot, map and particles
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

        %find smallest error and assign sigma to it
        [lost_values , min_index]= sortrows(dist,1);
        sigma = lost_values(1,1);

        %update readings_lost with sorted values
        readings_lost = lost_values(1,2);

        %reset best flag
        best_flag = 1.6;

        %normalize weights
        weights = weight./sum(weight);

        %check for nice cluster 
        %TODO check in lab
        if((3.5*sigmax < 0.3*max_dim)&&(3.5*sigmay < 0.3*max_dim))||(path_plan)
            %set blobcheck and sort weights to calculate mean particle
            blobcheck = 1;
            sigma_sense = 0.1 + noiselevel/4;
            sort_weights = sort([weights' [1:num]']);

            %take top 75% of indices from weights 
            index_average = sort_weights(round(end/4):end,2);
            %calculate mean position 
            mean_pos = mean(par_pos(index_average,:));

            %first sort angles by ranging from smallest to largest 
            sort_ang = sort(par_ang(index_average));
            range_ang = sort_ang(end) - sort_ang(1);

            %if greater than pi, angles are present in both first and fourth
            %quadrant 
            %check for values in fourth quadrant and set to negative
            if (range_ang>pi)
               index_to_change =  find((sort_ang>3/2*pi).*(sort_ang)>0);
               sort_ang(index_to_change) = sort_ang(index_to_change)-2*pi;
            end

            %calcuate mean angle from sorted vector 
            mean_ang = mean(sort_ang);
            mean_particle.setBotPos(mean_pos);
            mean_particle.setBotAng(mean_ang);

            %peform ultraScan for mean particle
            [parScan, crosspoints] = mean_particle.ultraScan();

            [value, index_to_check]  = sort(abs(crosspoints(1: readings/2,1)-mean_pos(1,1)));
            par_sense = parScan([mod(index_to_check(1)-1 + readings, readings)+1 mod(index_to_check(1)-1 + readings/4 + readings, readings)+1 mod(index_to_check(1)-1 + readings/2 +  readings, readings)+1 mod(index_to_check(1)-1 + + readings*3/4 + readings, readings)+1]);
            bot_sense = botScan([mod(index_to_check(1)-1 + readings, readings)+1 mod(index_to_check(1)-1 + readings/4 + readings, readings)+1 mod(index_to_check(1)-1 + readings/2 +  readings, readings)+1 mod(index_to_check(1)-1 + + readings*3/4 + readings, readings)+1]);
            bad_meas = find((bot_sense < 0) | (bot_sense > 254)| (bot_sense > saturation_value));
            readings_lost = 4 - length(bad_meas);

            %remove the bad values from the particle and bot vector
            bot_sense(bad_meas) = [];    
            par_sense(bad_meas) = [];

            %update sigma with mean particle readings
            sigma = sum((abs(par_sense - bot_sense)));

            %increase lost criteria 
            best_flag = 3;
            damping = 0;
        else
            blobcheck = 0;
            sigma_sense = 4 + noiselevel/2;
            damping = 0.015;
            damping = 0;
        end
    else
        %set all weights equally 
        weights = ones(1,num);
        %%check if you need normalization
    end
       
    %% Write code to check for convergence  
    if(6*sigmax < 0.25*max_dim)&&(6*sigmay < 0.25*max_dim)
        sort_weights = sort([weights' [1:num]']);
        index_average = sort_weights(round(end/4):end,2);
        
        mean_pos = mean(par_pos(index_average,:));
        sort_ang = sort(par_ang(index_average));
        range_ang = sort_ang(end) - sort_ang(1);
        
        if (range_ang>pi)
           index_to_change =  find((sort_ang>3/2*pi).*(sort_ang)>0);
           sort_ang(index_to_change) = sort_ang(index_to_change)-2*pi;
        end
        
        mean_ang = mean(sort_ang);
        mean_particle.setBotPos(mean_pos);
        mean_particle.setBotAng(mean_ang);
        
        [parScan, crosspoints] = mean_particle.ultraScan();
        
        [value index_to_check]  = sort(abs(crosspoints(1: readings/2,1)-mean_pos(1,1)))
        par_sense = parScan([mod(index_to_check(1)-1 + readings, readings)+1 mod(index_to_check(1)-1 + readings/4 + readings, readings)+1 mod(index_to_check(1)-1 + readings/2 +  readings, readings)+1 mod(index_to_check(1)-1 + + readings*3/4 + readings, readings)+1]);
        bot_sense = botScan([mod(index_to_check(1)-1 + readings, readings)+1 mod(index_to_check(1)-1 + readings/4 + readings, readings)+1 mod(index_to_check(1)-1 + readings/2 +  readings, readings)+1 mod(index_to_check(1)-1 + + readings*3/4 + readings, readings)+1]);
        bad_meas = find((bot_sense < 0) | (bot_sense > 254)| (bot_sense > saturation_value));
        readings_num = 4 - length(bad_meas);
        
        %remove the bad values from the particle and bot readings vector
        bot_sense(bad_meas) = [];       
        par_sense(bad_meas) = [];

        error_conv = sum((abs(par_sense - bot_sense)))
        
        %if error is small apply first check for convergence
        if(sum((abs(par_sense - bot_sense))) < (0.1+noiselevel*0.6)*readings_num)
               if(path_plan)
                   %make error tighter for a better and more accurate
                   %convergence 
                   if(sum((abs(par_sense - bot_sense)))< (0.1+noiselevel*0.4)*readings_num)
                        localise = 1;
                        mean_particle.drawBot(30,'g'); %draw robot with line length 30 and green
                        
                        position = mean_particle.getBotPos(); 
                        disp(['Robot Estimate: [x y angle]: [' num2str(position(1), 3) ' ' num2str(position(2), 3) ' ' num2str(mean_particle.getBotAng(), 5) ']' ]);
                        
                        [moveCmd turnCmd optimal] = Astar_visbil(mean_pos,mean_ang, adjacency,reduced,botSim,stepsize,target, path_dist,map);
                        
                        %check for last 2 movements and turn and move robot
                        %and mean particle
                        if(length(moveCmd) <= 2)
                             for looping = 1: length(moveCmd)
                                turn = turnCmd(looping) - mean_particle.getBotAng();
                                
                                turning(speed,turn);
                                driving(speed,moveCmd(looping));
                                
                                mean_particle.turn(turn);
                                mean_particle.move(moveCmd(looping));
                                position = mean_particle.getBotPos();
                                disp(['Robot Estimate: [x y angle]: [' num2str(position(1), 3) ' ' num2str(position(2), 3) ' ' num2str(mean_particle.getBotAng(), 5) ']' ]);
                             end
                             converged = 1; 
                        end
                   else
                       localise = 0;
                   end
               else
                   converged = 1;
               end
        else
            localise = 0;
        end
    else
        localise = 0;
    end
    
    %% Write code for resampling your particles
    %Stochastic resampling method
    if(localise == 0)&&(flag ~=4)
        current_weight = weights(1);
        rand_offset = rand(1) / num;
        i = 1;
        for m=1:num
            %compute upperbound
            upperbound = rand_offset + (m-1) * (1 / num);
            %while upperbound is smaller than cumulative weight, add next
            %weight until examined weight is less than upperbound
            while (upperbound > current_weight)
                i = i + 1;
                current_weight = current_weight + weights(i);
            end

            %set particles to chosen particles 
            chosen_particles(m).setBotPos(particles(i).getBotPos());
            chosen_particles(m).setBotAng(particles(i).getBotAng());
        end

        %iterate and set newly updated particles with chosen particles
        %position and angle + random noise to form cluster around chosen particles
        for i=1:num
            particles(i).setBotPos(chosen_particles(i).getBotPos + [normrnd(0,cloudsize) normrnd(0,cloudsize)]);
            particles(i).setBotAng(chosen_particles(i).getBotAng + normrnd(0,cloudangle));
        end
    end 
    
    %Print position and angle of best particle
    if(localise == 0)
        position = particles(min_index).getBotPos(); 
        disp(['Robot Estimate: [x y angle]: [' num2str(position(1), 3) ' ' num2str(position(2), 3) ' ' num2str(particles(min_index).getBotAng(), 5) ']' ]);
    end
end
end
