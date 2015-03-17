function [botSim, mean_pos mean_ang] = localise_today(botSim,map,target)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

%% setup code
%you can modify the map to take account of your robots configuration space
%give instructions to botsim object
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);
draw = 1
%generate some random particles inside the map
num =400; % number of particles
particles(num,1) = BotSim; %how to set up a vector of objects
for i=1:num
    particles(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
    particles(i).randomPose(10); %spawn the particles in random locations
end

%% Localisation code
maxNumOfIterations = 50;
n = 0;
converged =0; %The filter has not converged yet
dist = zeros(1,num);
weight = zeros(1,num);
sigma = 60;
botScan = botSim.ultraScan(); %get a scan from the real robot.
count = 0;
best_particle = BotSim(modifiedMap);
damping_factor = 1;
sigma_sense = 2;
sigma_sense = 1.5;
damping = 1/(num*700000);
damping = 10e-20;
%damping = 1/(num*1000000);
while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    
    %face shift check
    
    
    n = n+1; %increment the current number of iterations

    
    %% Write code to decide how to move next
    %% here they just turn in cicles as an example
    %% move=[step 0;step 0;step 0;step 90;step 0;step 0;step 90; step 0; step 0;step 0]; % sequence of motions
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
      if(botScan(1) < 10)||(botScan(2) < 5)||(botScan(6) < 5)
        if(botScan(2) < 10)
            botSim.turn(-turn);
            flag = 1;
        elseif(botScan(6) < 10)
            botSim.turn(turn);
            flag = 2;
        else
            botSim.turn(pi);
            flag = 3;
        end
      end

    if(flag == 0) && (mod(n,5) == 0)
        turn = ((-1)^(n))*pi/4;
        botSim.turn(turn);
        flag = 4;
    end
    %%botSim.turn(turn); %turn the real robot.  
    botSim.move(move); %move the real robot. These movements are recorded for marking 
    for i =1:num %for all the particles. 
        %%particles(i).turn(particles(i).getBotAng); %turn the particle in the same way as the real robot
        if(flag == 1)
            particles(i).turn(-turn);
        elseif (flag == 2)
            particles(i).turn(turn);
        elseif (flag == 3)
            particles(i).turn(pi);
        elseif (flag == 4)
            particles(i).turn(turn); 
        end
        
        %%ADD FLAG 5 AND TURN IF CONVERGED, THEN WHEN CONVERGE SET BACK TO
        %%0
        
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
    sigmax = std(par_pos(:,1))
    sigmay = std(par_pos(:,2))
    sigmaang  = std(par_ang)
    
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
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    botScan = botSim.ultraScan(); %get a scan from the real robot.
    %min_bot_scan = min(botScan);
    %% Write code for updating your particles scans
    cutoff = 31 - n;
    
    if(cutoff < 15)
        cutoff = 15;
    end
%%check measurements using p(z|r) in ideal case std is v small in reality
%%we can add sensor noise

    for i=1:num  
        %%check if particle outside the map
        if(particles(i).insideMap == 0) || (sigma > 60)&&(n>1)
            
            
            particles(i).randomPose(10);
            rand_pos = particles(i).getBotPos;
            checkx = find(round(par_pos(:,1)) == round(rand_pos(:,1)));
            if (~isempty(checkx))
                checky = find(round(par_pos(checkx,2)) == round(rand_pos(:,2)));
                if (~isempty(checky))
                    particles(i).randomPose(10);
                end
            end
            
           
                
            
            
            %particles(i).setBotPos(best_particle.getBotPos + rand(1)*best_particle_scan.*[cos(best_particle.getBotAng) sin(best_particle.getBotAng)]);
            %particles(i).setBotAng(best_particle.getBotAng + rand(1)*2*pi);
            
            %%n = 1;
        end
        
        parScan = particles(i).ultraScan();
        readings = 6;
        for j =0 :readings-1
            check_orient(j+1) = sqrt(sum((botScan - circshift(parScan,j)) .^ 2)); 
        end
        [lowest, best] = min((check_orient));
        %dist(i) = sqrt(sum((botScan - circshift(parScan,best - 1)) .^ 2)); 
        if(lowest <14)
            dist(i) = lowest;
            particles(i).turn(-2*pi/readings * (best-1));      
            parScan = circshift(parScan,best-1);
        end
        error = normpdf((botScan - parScan),0,sigma_sense);
        weight(i) = prod(error) + damping;
        dist(i) = sqrt(sum((botScan - parScan) .^ 2));

        
        %weight(i) = normpdf(dist(i),0,sigma*2);
      
    end
    
   
    [sigma,best] = min(dist); 
    if(sigma>60)
        sop = 0;
    end
    
    %% Write code for scoring your particles
    %%normalize weights
    weights = weight / sum(weight);
    
    %% Write code to check for convergence  
    %%choose best particle
    %%
    if(6*sigmax < 9)&&(6*sigmay < 9)&&(6*sigmaang < 50/180*pi)
        
        
        
   
        
        if botSim.debug()
            bot_pos = botSim.getBotPos
            bot_ang = botSim.getBotAng
        end
        mean_pos = mean(par_pos)
        mean_ang = mean(par_ang)
        
         converged = 1; 
           disp(n);
           
       
        
    end
    
    %% Write code for resampling your particles
    Q = cumsum(weights);
    t = rand(1,num+1);
    T = sort(t);
    T(num+1) = 1;
    i = 1; j = 1;
    Index = zeros(1,num);
    while(i <= num && j <= num)
        if(T(i) < Q(j))
            Index(i) = j;
            i = i+1;
        else
            j = j+1;
        end
    end
%     iSelect = rand(num,1); %?nd the particle that corresponds to each y value (just a look up) 
%     Index = interp1(Q,1:num,iSelect,'nearest','extrap');
    %%respawn particles
    temp_particles = particles;
     
    cloudsize = 1; %radius of the respawn cloud about previous particles
    cloudangle = pi/36; %angle of respawned particles
%     if(n > 10)
%         cloudangle = pi/18;
%     end
%     
%     if(sigma > 7) && (sigma < 20)
%         cloudsize = 4;
%     end

    for i=1:num
        index = Index(i);
        %max_x = max(map(:,1));
        %max_y = max(map(:,2));
         
        %min_x = min(map(:,1));
        %min_y = min(map(:,2));
        
        if(index > 0)
            %%pos = particles(index).getBotPos;
            %%pos_x = pos(:,1);
            %%pos_y = pos(:,2);

            %%if((pos_x < max_x && pos_y < max_y) && (pos_x > min_x && pos_y > min_y))
                temp_particles(i).setBotPos(particles(index).getBotPos + [normrnd(0,cloudsize) normrnd(0,cloudsize)]);
                temp_particles(i).setBotAng(particles(index).getBotAng + normrnd(0,cloudangle));
            %%end
        end
    end
            
    particles = temp_particles;

    %%calculate mean pos and ang of particles
%     sum_pos = [0 0];
%     sum_ang = 0;
%      for i =1:num
%     sum_pos = sum_pos + particles(i).getBotPos;
%     wrapped_angle = particles(i).getBotAng;
%     if (wrapped_angle>0)
%       wrapped_angle =  mod(wrapped_angle,2*pi);
%     else
%         wrapped_angle =  mod(wrapped_angle,-2*pi);
%         wrapped_angle =  wrapped_angle + 2*pi;
%     end
%         sum_ang = sum_ang + wrapped_angle;
%     end
%     
%     mean_pos = sum_pos / num
%     mean_ang = sum_ang / num
    
    

    %% Write code to take a percentage of your particles and respawn in randomised locations (important for robustness)	

    
    %disp(count);
    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
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
