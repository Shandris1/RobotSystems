function [botSim, mean_pos mean_ang] = localise_final2(botSim,map,target)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

%% setup code
%you can modify the map to take account of your robots configuration space
%give instructions to botsim object
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);
draw = 1
%generate some random particles inside the map
num = 400; % number of particles
particles(num,1) = BotSim; %how to set up a vector of objects
mean_particle = BotSim(modifiedMap);
mean_particle.setScanConfig(botSim.generateScanConfig(8));
chosen_particles(num,1) = BotSim(modifiedMap);   
count_conv = 0;
readings = 8;
for i=1:num
    particles(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
    chosen_particles(i) = BotSim(modifiedMap);
    chosen_particles(i).setScanConfig(botSim.generateScanConfig(readings));
    particles(i).randomPose(5); %spawn the particles in random locations
    particles(i).setScanConfig(botSim.generateScanConfig(readings));
end

%% Localisation code
maxNumOfIterations = 50;
n = 0;
converged =0; %The filter has not converged yet
dist = zeros(1,num);
weight = zeros(1,num);
sigma = [60 0];
Sigma = 40;
sigma_sense = 2;
damping = 0;
%damping = 1/(num*1000000);
converged_candidate = 0;
countin_reinital = 0;
cloudsize = 1; %radius of the respawn cloud about previous particles
botfound = 0;
cloudangle = pi/36; %angle of respawned particles
closest = 1.2;
botScan = botSim.ultraScan(); %get a scan from the real robot.
wall = 0;
while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    
    n = n+1; %increment the current number of iterations

    %% Write code to decide how to move next
    move = 5;
    turn = pi/2;
    %turn = pi/3;
    flag = 0;
    
%     if(botScan(1) < 10)
%         if(botScan(2) < 10)
%             botSim.turn(-turn);
%             flag = 1;
%         elseif(botScan(6) < 10)
%             botSim.turn(turn);
%             flag = 2;
%         else
%             botSim.turn(pi);
%             flag = 3;
%         end
%     end
%     if(botScan(1) < 10)||(botScan(2) < 5)||(botScan(6) < 5)
%         if(botScan(2) < 10)
%             botSim.turn(-turn);
%             flag = 1;
%         elseif(botScan(6) < 10)
%             botSim.turn(turn);
%             flag = 2;
%         else
%             if(botScan(2) >= botScan(6))
%                 turn = -7/12*pi;
%             else
%                 turn = 7/12*pi; 
%             end
%             botSim.turn(turn);
%             flag = 3;
%         end
%     end
    %pause(0.5);
     if (botScan(1) < 10)
         botSim.turn(turn);
         wall = 1;
         flag = 1;
%      elseif (wall == 1 && botScan(3) < 10)
%          botSim.turn(-turn);
%          flag = 2;
     end
     
     if (converged_candidate == 1)&&(flag==0)
        flag = 5;
        turn = pi/6;
        botSim.turn(turn);  %%convergence test turn
     end
    
%       
%     if(flag == 0) && (mod(n,3) == 0)
%         [max_val, max_ang_ind] = max(botScan);
%         turn = ((-1)^(n))*pi/4;
%         turn = 2*pi/readings*(max_ang_ind-1)+rand(1)*pi/12;
%         botSim.turn(turn);
%         flag = 4;
%     end

    %%botSim.turn(turn); %turn the real robot.  
    botSim.move(move); %move the real robot. These movements are recorded for marking 

    for i =1:num %for all the particles. 
        %%particles(i).turn(particles(i).getBotAng); %turn the particle in the same way as the real robot
        
        if (flag == 1)
            particles(i).turn(turn);
        elseif (flag == 2)
            particles(i).turn(-turn);
        elseif (flag == 5)
            particles(i).turn(turn);
            
%         if(flag == 1)
%             particles(i).turn(-turn);
%         elseif (flag == 2)
%             particles(i).turn(turn);
%         elseif (flag == 3)
%             particles(i).turn(turn);
%         elseif (flag == 4)
%             particles(i).turn(turn); 
%         elseif (flag == 5)
%             particles(i).turn(turn);  
%         end
        
        particles(i).move(move); %move the particle in the same way as the real robot
        
        par_pos(i,:) =  particles(i).getBotPos;
        par_ang(i) = particles(i).getBotAng;
        
        if (par_ang(i)>0)
            par_ang(i) =  mod(par_ang(i),2*pi);
         else
            par_ang(i) =  mod(par_ang(i),-2*pi);
            par_ang(i) =  par_ang(i) + 2*pi;
        end
    
    end
    
    sigmax = std(par_pos(:,1));
    sigmay = std(par_pos(:,2));
    sigmaang  = std(par_ang);
    
    botScan = botSim.ultraScan(); %get a scan from the real robot.
    %min_bot_scan = min(botScan);
    %% Write code for updating your particles scans
    %%check measurements using p(z|r) in ideal case std is v small in reality
    %%we can add sensor noise

    for i=1:num  
        %%check if particle outside the map
        if (sigma(1) > 1.3)&&(n>1)&&(sigma(2) < 1.3)  || (particles(i).insideMap == 0) 
            particles(i).randomPose(0);

            rand_pos = particles(i).getBotPos;
            checkx = find(round(par_pos(:,1)) == round(rand_pos(:,1)));
            if (~isempty(checkx))
                checky = find(round(par_pos(checkx,2)) == round(rand_pos(:,2)));
                if (~isempty(checky))
                    particles(i).randomPose(0);
                end
            end
        end
 
        parScan = particles(i).ultraScan();
        
        for j =0 :readings-1
            %check_orient(j+1)  = max((abs(circshift(parScan,j) - botScan))./botScan);
            %check_orient2(j+1) = sqrt(sum((botScan - circshift(parScan,j)) .^ 2)); 
            check_orient(j+1) = sum((abs(circshift(parScan,j) - botScan))./botScan);
        end
        [lowest, best] = min((check_orient));
        %dist(i) = sqrt(sum((botScan - circshift(parScan,best - 1)) .^ 2)); 
        if(lowest < closest)
            particles(i).turn(-2*pi/readings * (best-1));      
            parScan = circshift(parScan,best-1);
        end
        
        %error = normpdf((botScan - parScan),0,sigma_sense);
        %weight(i) = error;
        %weight(i) = normpdf(sqrt(sum((botScan - parScan) .^ 2)),0,sigma_sense);
        
        %Sig=7*eye(length(botScan));
        %Sig=7*ones(1,length(botScan));
        %likelihood = mvnpdf(mean(botScan-parScan)', 0, Sig);
        %weight(i) = likelihood + damping;
        
        %eucl = sqrt(sum((botScan - parScan) .^ 2));
        %k = sum(abs(botScan - parScan));
        %weight(i) = (exp(1) .^ -(k)) + damping;
        
        error = normpdf((botScan - parScan),0,sigma_sense);
        weight(i) = sum(error) + damping;
        
        %Sig=sigma_sense*eye(length(botScan));
         
        %Sig=Sigma*eye(length(botScan));
        %likelihood = mvnpdf(parScan',botScan',Sig);
        %likelihood = mvnpdf((abs(botScan - parScan)), 0, Sig);
        %weight(i) = likelihood;

        dist(i)= sum((abs(parScan - botScan))./botScan);
        %weight(i) = normpdf(dist(i),0,sigma*2);
      
    end
    
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
            drawnow;
         end
    end
    
    sigma(2)=sigma(1);
    [sigma(1),best] = min(dist); 
    if(sigma(1)>1.3) && (sigma(2)<1.3)
       countin_reinital = countin_reinital + 1
       reinitialise_flag = 1;
    end
    
    %figure
    %plot3(par_pos(:,1),par_pos(:,2),weight,'x');
     
  
    %% Write code for scoring your particles
    %%normalize weights
    weights = weight./sum(weight);
    %weights = weights + damping;
    %weights = weights./sum(weights);
    

    %% Write code to check for convergence  
    if(6*sigmax < 25)&&(6*sigmay < 25)&&(6*sigmaang < 100/180*pi)
        botfound = 1;
        if botSim.debug()
            bot_pos = botSim.getBotPos
            bot_ang = botSim.getBotAng
        end
        mean_pos = mean(par_pos);
        no_outlier =sort(par_ang);
        mean_ang = mean(no_outlier(round(0.25*length(no_outlier)):round(0.75*length(no_outlier))));
        mean_particle.setBotPos(mean_pos);
        mean_particle.setBotAng(mean_ang);
        measurement = mean_particle.ultraScan();
        %error_conv = mean((abs(measurement - botScan))./botScan)
        error_conv = sum((abs(measurement - botScan))./botScan)
        if(sum((abs(measurement - botScan))./botScan)< 0.5)
             if converged_candidate == 1
                 converged = 1;
                 n
             else
                 converged_candidate = 1;
                 count_conv = count_conv  + 1
             end
         end
  
    else
        converged_candidate = 0;
    end
    
    %% Write code for resampling your particles
    %% Select with Replacement 
%     Q = cumsum(weights);
%     t = rand(1,num);
%     T = sort(t);
%     i = 1; j = 1;
%     Index = zeros(1,num);
%     while(i <= num && j <= num)
%         if(T(i) < Q(j))
%             Index(i) = j;
%             i = i+1;
%         else
%             j = j+1;
%         end
%     end
 
    c = weights(1);
    r = rand(1) / num;
    i = 1;
    for m=1:num
        U = r + (m-1) * (1 / num);
        while (U > c)
            i = i + 1;
            c = c + weights(i);
        end
        particles(i).getBotPos()
        chosen_particles(m).setBotPos(particles(i).getBotPos());
        chosen_particles(m).setBotAng(particles(i).getBotAng());
    end
    % index = Index(i);
    %if(index > 0)
    for i=1:num
        particles(i).setBotPos(chosen_particles(i).getBotPos + [normrnd(0,cloudsize) normrnd(0,cloudsize)]);
        particles(i).setBotAng(chosen_particles(i).getBotAng + normrnd(0,cloudangle));
        disp(10);
        chosen_particles(i).getBotPos()
    end
    x = 1;      
    %particles = chosen_particles;

%     %% Drawing
%     %only draw if you are in debug mode or it will be slow during marking
%     if botSim.debug()
%         hold off; %the drawMap() function will clear the drawing when hold is off
%         botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
%         botSim.drawBot(30,'g'); %draw robot with line length 30 and green
%         for i =1:num
%             if(particles(i).insideMap == 1)
%                 particles(i).drawBot(3); %draw particle with line length 3 and default color
%             end
%         end
%         drawnow;
%     end
end
end
