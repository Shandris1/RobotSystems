function [mean_particle, particles, mean_pos mean_ang] = localisation(botSim,mean_particle,chosen_particles,particles,map,target,readings, noiselevel,num,path_plan,max_dim,moveCmd,turnCmd,optimal,localise,stepsize,adjacency,reduced,path_dist)
%If readings are not divisible by 4, the wall-following program will crash - VERY IMPORTANT!


%% parameters that are used for tuning the performance
draw = 1;
sigma = 60;
sigma_sense = (10 + noiselevel/2);
blobcheck= 0;
damping = 1/(readings*5);%this is amazing for sensor noise but takes too long
damping = 1/(readings*7);%last version
damping = 0;
cloudsize = 0.5 + noiselevel*0.1; %radius of the respawn cloud about previous particles
cloudangle = 4/180*pi; % 4 degrees angle of respawned particles
closest = 8*readings;%5cm per reading
wall_follow_dist = 15 + noiselevel;


%% Localisation code
maxNumOfIterations = 500;
n = 0;
converged =0; %The filter has not converged yet
dist = zeros(1,num);
weight = zeros(1,num);
search_long = 1;
flag_random = 0;
random_move = 1;
%workspace = rand_wrokspace(map,num);
noise = noiselevel + 7;

botScan = botSim.ultraScan(); %get a scan from the real robot.

while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    
    n = n+1; %increment the current number of iterations
    
    %% Write code to decide how to move next
    flag_move = 0;
    flag_turn = 0;
    max_found = 0;
    
    %find furthest wall
    if(path_plan == 0)
        if(search_long)
           if(n>1)
               [max_val, max_ang_ind] = max(botScan);
               if(max_ang_ind == (readings/2 +1))
                   temp = botScan;
                   temp(max_ang_ind) =0;
                   [max_val, max_ang_ind] = max(temp);

               end
           else
                [max_val, max_ang_ind] = max(botScan);

           end

           turn = 2*pi/readings*(max_ang_ind-1)+rand(1)*pi/9;
           turn = 2*pi/readings*(max_ang_ind-1)+rand(1)*pi/18;
           botSim.turn(turn);
           search_long = 0;
           move = random_move;
           max_found = 1;
           flag_turn = 1;
        end
        
        if(botScan(1) > wall_follow_dist)||(max_found)
            move = random_move;
            botSim.move(move);
            search_long = 0;
            max_found = 0;
            flag_move = 1;
        else
            search_long = 1;
        end
    else     
        if (localise == 0)
            turn = pi/36 + randn(1)*pi/36;
            botSim.turn(turn);
            flag_turn = 1;
       else
           turn_cmd = turnCmd(1);
           move = moveCmd(1);
           turn = turn_cmd - mean_particle.getBotAng();

           botSim.turn(turn);
           botSim.move(move);

           flag_move = 1;
           flag_turn = 1;
       end 
    end
 
    %assign the same behaviour to the particles
    for i =1:num 
        if (flag_turn == 1)
            particles(i).turn(turn);
        end
        if (flag_move == 1) 
            particles(i).move(move); 
        end
        
        par_pos(i,:) =  particles(i).getBotPos;
        par_ang(i) = particles(i).getBotAng;
        
        par_ang(i) = mod((par_ang(i)+2*pi),2*pi);
    end
    
    % get the standart deviation for X Y and theta
    sigmax = std(par_pos(:,1));
    sigmay = std(par_pos(:,2));
    sigmaang  = std(par_ang);
    
    %get readings from the bot
    botScan = botSim.ultraScan(); 

    %% Write code for updating your particles scans
    counting = 0;
    curr_flag = 0;
    for i=1:num  
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
        % perform a check to verify if the robot is lost
        % if lost randomly spread all particles around the map
        if (sigma > (noiselevel*0.8+ 5)*readings)&&(n>3)&&(flag_random == 0)
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
 
        % get ultrasonic readings for all particles
        parScan = particles(i).ultraScan();

        for j =0 :readings-1
            check_orient(j+1) = sum((abs(circshift(parScan,j) - botScan)));
        end
        [lowest, best] = min((check_orient));
        
        if(lowest < closest)
            particles(i).turn(-2*pi/readings * (best-1));      
            parScan = circshift(parScan,best-1);
            counting = counting +1;
            circ_ind(counting) = i;
        end

        %get the sum of the differences between bot readings and particle
        %readings in order to assign weigths to particles
        error = sum(abs(botScan - parScan));
        weight(i) = exp((-(error)^2)/(2*(sigma_sense*readings*0.6)^2));
        
        % assign the distance to be the sum of the difference
        % between bot readings and particle readings
        dist(i)= sum((abs(parScan - botScan)));
      
    end
    
    if(curr_flag == 1)
        flag_random = 4;
    elseif(flag_random > 0) && (flag_random <= 4)
        flag_random = flag_random - 1;
    end
 
    %% Draw bot and particles
    if(draw == 1)   
         if botSim.debug()
            hold off; %the drawMap() function will clear the drawing when hold is off
            botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
            plot(target(:,1),target(:,2),'x','MarkerSize', 40);
            botSim.drawBot(30,'r'); %draw robot with line length 30 and green
            if(path_plan)
                mean_particle.drawBot(60,'g');
            end
            for i =1:num
                %if(particles(i).insideMap == 1)
                    particles(i).drawBot(3); %draw particle with line length 3 and default color
               % end
            end
%              for i =1:counting
%                 if(particles(i).insideMap == 1)
%                     particles(circ_ind(i)).drawBot(15,'r'); %draw particle with line length 3 and default color
%                 end
%             end
            drawnow;
         end
    end
    
    
    % assign sigma to be the minimum distance between bot and particles
    [sigma] = min(dist);
    
    if(6*sigmax < 0.3*max_dim)&&(6*sigmay < 0.3*max_dim)
        blobcheck = 1;
        sigma_sense = 2 + noiselevel/4;
    else
        blobcheck = 0;
        sigma_sense =10 + noiselevel/2;
    end
    
    %%normalize weights
    weights = weight./sum(weight);
    
    %% Write code to check for convergence  
    if(6*sigmax < 0.15*max_dim)&&(6*sigmay < 0.15*max_dim)
%         if botSim.debug()
%             bot_pos = botSim.getBotPos;
%             bot_ang = botSim.getBotAng;
%         end
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
        
        %get the mean position and angle
        mean_ang = mean(sort_ang);
        mean_particle.setBotPos(mean_pos);
        mean_particle.setBotAng(mean_ang);
        mean_particle.drawBot(60,'g');
        measurement = mean_particle.ultraScan();
        %error_conv = sum((abs(measurement - botScan))./botScan)
        if(sum((abs(measurement - botScan)))< (3+noiselevel)*readings)
               if(path_plan)
                    localise = 1;
                    mean_particle.drawBot(60,'g'); %draw robot with line length 30 and green
                    position = mean_particle.getBotPos();
                    disp(['Robot Estimate: [x y angle]: [' num2str(position(1), 3) ' ' num2str(position(2), 3) ' ' num2str(mean_particle.getBotAng(), 5) ']' ]);
                    
                    [moveCmd turnCmd optimal] = Astar_visbil(mean_pos,mean_ang, adjacency,reduced,botSim,stepsize,target, path_dist,map);

                    if(length(moveCmd) == 2)
                        turn = turnCmd(1) - mean_particle.getBotAng();
                        botSim.turn(turn);
                        botSim.move(moveCmd(1));
                        
                        mean_particle.turn(turn);
                        mean_particle.move(moveCmd(1));
                        position = mean_particle.getBotPos();
                        disp(['Robot Estimate: [x y angle]: [' num2str(position(1), 3) ' ' num2str(position(2), 3) ' ' num2str(mean_particle.getBotAng(), 5) ']' ]);
                        
                        turn = turnCmd(2) - mean_particle.getBotAng();
                        botSim.turn(turn);
                        botSim.move(moveCmd(2));
                        
                        mean_particle.turn(turn);
                        mean_particle.move(moveCmd(2));
                        
                        converged = 1;
                    end
               else
                   converged = 1;
               end
        else
            localise = 0;
        end
    else
        if(path_plan)
            localise = 0;
        end
    end
    
    %% Write code for resampling your particles
    c = weights(1);
    r = rand(1) / num;
    i = 1;
    for m=1:num
        %find high weigths
        U = r + (m-1) * (1 / num);
        while (U > c)
            i = i + 1;
            c = c + weights(i);
        end

        % create a vector of particles that scored high
        chosen_particles(m).setBotPos(particles(i).getBotPos());
        chosen_particles(m).setBotAng(particles(i).getBotAng());
    end

    % resample particles only around good particles in order to form a
    % cluster
    % use a varying parameter for both cluster size and angle
    for i=1:num
        particles(i).setBotPos(chosen_particles(i).getBotPos + [normrnd(0,cloudsize) normrnd(0,cloudsize)]);
        particles(i).setBotAng(chosen_particles(i).getBotAng + normrnd(0,cloudangle));
    end
    
    if(localise == 0)
        [min_val min_index] = min(dist);
        position =particles(min_index).getBotPos();
        %fprintf('RobotEstimate[x y angle]: %.3f %.3f %.3f',[ particles(min_index).getBotPos(1) particles(min_index).getBotPos(2) particles(min_index).getBotAng]);
        disp(['Robot Estimate: [x y angle]: [' num2str(position(1), 3) ' ' num2str(position(2), 3) ' ' num2str(particles(min_index).getBotAng(), 5) ']' ]);
    end
end
end
