function [botSim] = localise(botSim,map,target)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

%% setup code
%you can modify the map to take account of your robots configuration space
%give instructions to botsim object
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);
botSim.setScanConfig(botSim.generateScanConfig(10));

%generate some random particles inside the map
num =200; % number of particles
particles(num,1) = BotSim; %how to set up a vector of objects
for i=1:num
    particles(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
    particles(i).setScanConfig(particles(i).generateScanConfig(10));
    particles(i).randomPose(10); %spawn the particles in random locations
end

%% Localisation code
maxNumOfIterations = 50;
n = 0;
converged =0; %The filter has not converged yet
dist = zeros(1,num);
weight = zeros(1,num);
sigma1 = 70;
sigma = 70;
botScan = botSim.ultraScan(); %get a scan from the real robot.

while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations
    
    %% Write code to decide how to move next
    %% here they just turn in cicles as an example
    %% move=[step 0;step 0;step 0;step 90;step 0;step 0;step 90; step 0; step 0;step 0]; % sequence of motions
    move = 1;
    turn = pi/2;
    flag = 0;
    
    if(botScan(1) < 10)
        botSim.turn(turn);
        flag = 1;
    end
    %%botSim.turn(turn); %turn the real robot.  
    botSim.move(move); %move the real robot. These movements are recorded for marking 
    for i =1:num %for all the particles. 
        %%particles(i).turn(particles(i).getBotAng); %turn the particle in the same way as the real robot
        if(flag)
            particles(i).turn(turn);
        end
        
        particles(i).move(move); %move the particle in the same way as the real robot
    end
    
    botScan = botSim.ultraScan(); %get a scan from the real robot.
    
    %% Write code for updating your particles scans
    cutoff = 71 - n*2;
    
    if(cutoff < 15)
        cutoff = 15;
    end
    
    for i=1:num  
        %%check if particle outside the map
        if(particles(i).insideMap == 0) || (sigma1 > cutoff)
            particles(i).randomPose(10);
            n = 1;
        end
        
        parScan = particles(i).ultraScan();
        dist(i) = sqrt(sum((botScan - parScan) .^ 2));
        mean_error(i) = sum(abs(botScan - parScan));
        weight(i) = prod(normpdf((botScan-parScan),0,sigma));
           
    end
    
    %sigma = std(dist);
    sigma = mean(mean_error);
    [sigma1,best] = min(dist); 
    
    %% Write code for scoring your particles
    %%normalize weights
    weights = weight / sum(weight);
    
    %% Write code to check for convergence  
    %%choose best particle
    if(sigma1 < 2)
        final_pos = particles(best).getBotPos
        final_ang = particles(best).getBotAng
        
        if botSim.debug()
            bot_pos = botSim.getBotPos
            bot_ang = botSim.getBotAng
        end
        
        particles(best).drawBot(30,'r')
        converged = 1;
    end
    
    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
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
    
    %% Write code for resampling your particles
    Q = cumsum(weights);
    t = rand(num+1);
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
    
    %%respawn particles
    temp_particles = particles;
    cloudsize = 2; %radius of the respawn cloud about previous particles
    cloudangle = pi/18; %angle of respawned particles
    
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
%     for i =1:num
%         sum_pos = sum_pos + particles(i).getBotPos;
%         sum_ang = sum_ang + particles(i).getBotAng;
%     end
%     
%     mean_pos = sum_pos / num
%     mean_ang = sum_ang / num
    
    

    %% Write code to take a percentage of your particles and respawn in randomised locations (important for robustness)	

    
    
    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
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
end
