function [move_commands turn_commands optimal] = Astar(currentPos,currentAng,target,closed,stepsize,robot)
%check that it is divisble by max range

%to do
%convert rotation facing target
%convert x y positions to commnds for simultation
%include different step sizes

open = [];
% open format
% x|y|parentx|parenty|h|g|f


% container = [];
% container_1=[];
% container_2 =[];
% map = [map ;map(1,:)];


%psuedo code%

%closed has all the indices of the map which are our obstacles
%initialise grid with obstacles start and end position
% put start on open and initilise g = 0 and hx disance to target
%then go into loop(until goal is reached with lowest cost)<<
% remove current node from open and put to close
%find all successors
%update open list with successors
% find the min value in the open list
% go to find shortest path algo
%finally output path by getting to last node and moving up to all parent
%and storing route
range_x = max(closed(:,1))- min(closed(:,1)) + 1;
offsetx = min(closed(:,1));
range_y = max(closed(:,2))- min(closed(:,2)) + 1;
offsety = min(closed(:,2));
size_closed = size(closed,1);
closed = [closed zeros(size_closed,2)];

grid = ones(ceil(range_y/stepsize),ceil(range_x/stepsize));
%sub2ind(size(grid), range_y+offsety-closed(:,2), -offsetx+1+closed(:,1));
grid(sub2ind(size(grid), ceil((range_y+offsety-closed(:,2))/stepsize), ceil((-offsetx+1+closed(:,1))/stepsize) )) = -1;
%to do check if inital poistion is valid...
%put start on open
open_count = 1;
open(open_count,:) = [currentPos currentPos abs( (currentPos(1) + 1i*currentPos(2)) - (target(1) +1i*target(2))) 0 abs( (currentPos(1) + 1i*currentPos(2)) - (target(1) +1i*target(2))) ];
%shortest path method
condition =1;
xpos = currentPos(1);
ypos = currentPos(2);
xpar = currentPos(1);
ypar = currentPos(2);
gvalue = open(1,6);
test =0;
 open(1,:) =[];
 nosol = 0;
while(condition)
   
    size_closed = size_closed +1;
    open_count = size(open,1) +1;
    if((~isempty(xpos)))
    closed(size_closed,:) = [xpos ypos xpar ypar];
    grid(range_y+offsety-ypos,-offsetx+1+xpos) =-1;
    test = test+1;
    end
    %to do check if inital poistion is valid...
     
    child = succ( [xpos ypos gvalue], grid,offsetx,offsety, range_y,target,robot);
    if(~isempty(child))
    open(open_count: open_count + size(child,1)-1,:)= child;
    end
    xpos =[];
    ypos=[];
  
    if(~isempty(open))
    open = sortrows(open,7);
    xpos = open(1,1);
    ypos = open(1,2);
    xpar = open(1,3);
    ypar = open(1,4);
    gvalue = open(1,6);
    
    
    
    %remove open location wih same position
    [r,c,v] = find((open(:,1)==xpos)&(open(:,2)==ypos));
    open(r,:)=[];
    if(xpos== target(1))&&(ypos ==target(2))
        condition = 0;
        
    end
    
    else
        condition = 0; %no more cells to check
        nosol = 1;
    end
    
end
%figure 
%imshow(grid)
optimal=[];
inc = 1;
%get path backwards
if(~nosol)
    optimal(1,:) =[xpos ypos];
    while(~((xpar== currentPos(1))&&(ypar ==currentPos(2))))
        inc = inc +1;
        optimal(inc,:) =[xpar ypar];
        
        [row col vol] = find((closed(:,1)==xpar)&(closed(:,2)==ypar));
        xpar = closed(row,3);
        ypar = closed(row,4);
    end
    
    
end


optimalb = optimal(end:-1:1);
x_diff = diff([currentPos(1); optimalb(:,1)]);
y_diff = diff([currentPos(2); optimalb(:,2)]);



move_commands = abs(x_diff + 1j.*y_diff);
turn_commands = atan2(y_diff,x_diff);

%assuming map given in order
% 
% for i = 1: size(map,1)-1
%     dx = abs(map(i+1,1) - map(i,1));
%     dy = abs(map(i+1,2) - map(i,2));
%     if dx ==0
%         if map(i+1,2) > map(i,2)
%             container(2,:) =  map(i,2) - rob_size:abs(stepsize):map(i+1,2)+ rob_size;
%             container(1,:) =  map(i,1);
%             
%             
%         else
%             container(2,:) =  map(i,2)+rob_size:-abs(stepsize):map(i+1,2) - rob_size;
%             container(1,:) =  map(i,1);
%         end
%         if(rob_size>0)
%             for j=-rob_size/stepsize:rob_size/stepsize
%                 container_1(1,:) = (container (1,:) + j*stepsize);
%                 container_1(2,:)= container(2,:);
%                 container_2 = [ container_2 ; container_1'];
%             end
%         else
%                  container_2 = container';
%         end
%     else
%         
%         %grad = atan2(dy,dx);
%         if (dy>dx)
%             
%             if map(i+1,2) > map(i,2)
%             container(2,:) =  round(map(i,2)- abs(rob_size*cos(grad-pi/2))):round(abs(stepsize*cos(grad-pi/2))):round(map(i+1,2)+abs(rob_size*cos(grad-pi/2)));
%         elseif map(i+1,2) < map(i,2)
%             container(2,:) =  round(map(i,2)+abs(rob_size*cos(grad-pi/2))):round(-abs(stepsize*cos(grad-pi/2))):round(map(i+1,2)-abs(rob_size*cos(grad-pi/2)));
%             end
%         elseif(dy<dx)
%              if map(i+1,1) > map(i,1)
%                 container(1,:) =  round(map(i,1)- abs(rob_size*sin(grad-pi/2))):stepsize:round(map(i+1,1) + abs(rob_size*sin(grad-pi/2)));
%             else
%                 container(1,:) =  round(map(i,1)+ abs(rob_size*sin(grad-pi/2))):-stepsize:round(map(i+1,1) - abs(rob_size*sin(grad-pi/2)));
%              end
%             if map(i+1,2) > map(i,2)
%             
%                 container(2,:) =  round(map(i,2)- abs(rob_size*cos(grad-pi/2))):round(abs(stepsize*cos(grad-pi/2))):round(map(i+1,2)+abs(rob_size*cos(grad-pi/2)));
%             elseif map(i+1,2) < map(i,2)
%             container(2,:) =  round(map(i,2)+abs(rob_size*cos(grad-pi/2))):round(-abs(stepsize*cos(grad-pi/2))):round(map(i+1,2)-abs(rob_size*cos(grad-pi/2)));
%             
%         else
%             if map(i+1,1) > map(i,1)
%                 container(1,:) =  map(i,1)- rob_size:stepsize:map(i+1,1) + (rob_size);
%             else
%                 container(1,:) =  map(i,1)+ rob_size:-stepsize:map(i+1,1) - (rob_size);
%             end
%              container(2,:) =  map(i,2);
%         end
% %             
% %         if map(i+1,1) > map(i,1)
% %             container(1,:) =  round(map(i,1)- abs(rob_size*sin(grad-pi/2))):round(abs(stepsize*sin(grad-pi/2))):round(map(i+1,1) + abs(rob_size*sin(grad-pi/2)));
% %         else
% %             container(1,:) =  round(map(i,1) + abs(rob_size*sin(grad-pi/2))):round(-abs(stepsize*sin(grad-pi/2))):round(map(i+1,1)- abs(rob_size*sin(grad-pi/2)));
% %         end
% %         if map(i+1,2) > map(i,2)
% %             i
% %             container(2,:) =  round(map(i,2)- abs(rob_size*cos(grad-pi/2))):round(abs(stepsize*cos(grad-pi/2))):round(map(i+1,2)+abs(rob_size*cos(grad-pi/2)));
% %         elseif map(i+1,2) < map(i,2)
% %             container(2,:) =  round(map(i,2)+abs(rob_size*cos(grad-pi/2))):round(-abs(stepsize*cos(grad-pi/2))):round(map(i+1,2)-abs(rob_size*cos(grad-pi/2)));
% %         else
% %             container(2,:) =  map(i,2);
% %         end
%         if(rob_size>0)
%             if dy>0
%                 for j=-rob_size/stepsize:rob_size/stepsize
%                     container_1(1,:) = (container (1,:) + j*stepsize);
%                     container_1(2,:)= container(2,:);
%                     container_2 = [ container_2 ; container_1'];
%                 end
%             else
%                  for j=-rob_size/stepsize:rob_size/stepsize
%                     container_1(2,:) = (container (2,:) + j*stepsize);
%                     container_1(1,:)= container(1,:);
%                     container_2 = [ container_2 ; container_1'];
%                  end
%                  
%              end
%          else
%              container_2 = container';
%          end
%         
%    end
%     closed = [ closed ; container_2];
%     container =[];
%     container_1=[];
%     container_2 = [];
% 
% 
% end
% 
% closed = unique(closed,'rows', 'stable');
% 
% plot(closed(:,1),closed(:,2), 'rx');
% end
