function [closed] = map_obstacle(map,rob_size,stepsize)


closed =[];
open = [];
container = [];
container_1=[];
container_2 =[];
map = [map ;map(1,:)];

%assuming map given in order

for i = 1: size(map,1)-1
    dx = abs(map(i+1,1) - map(i,1));
    dy = abs(map(i+1,2) - map(i,2));
    if dx ==0
        if map(i+1,2) > map(i,2)
            container(2,:) =  map(i,2) - rob_size:abs(stepsize):map(i+1,2)+ rob_size;
            container(1,:) =  map(i,1);
            
            
        else
            container(2,:) =  map(i,2)+rob_size:-abs(stepsize):map(i+1,2) - rob_size;
            container(1,:) =  map(i,1);
        end
        if(rob_size>0)
            for j=-rob_size/stepsize:rob_size/stepsize
                container_1(1,:) = (container (1,:) + j*stepsize);
                container_1(2,:)= container(2,:);
                container_2 = [ container_2 ; container_1'];
            end
        else
                 container_2 = container';
        end
    else
        m =dy/dx;
        grad = atan2(dy,dx);
        if (dy==0)
            
            if map(i+1,1) > map(i,1)
                container(1,:) =  map(i,1)- rob_size:stepsize:map(i+1,1) + (rob_size);
            else
                container(1,:) =  map(i,1)+ rob_size:-stepsize:map(i+1,1) - (rob_size);
            end
            container(2,:) =  map(i,2);
        elseif(dy<dx)
             if map(i+1,1) > map(i,1)
                container(1,:) =  round(map(i,1)- abs(rob_size*sin(grad-pi/2))):stepsize:round(map(i+1,1) + abs(rob_size*sin(grad-pi/2)));
            else
                container(1,:) =  round(map(i,1)+ abs(rob_size*sin(grad-pi/2))):-stepsize:round(map(i+1,1) - abs(rob_size*sin(grad-pi/2)));
             end
            if map(i+1,2) > map(i,2)
            
                container(2,:) = round([round(map(i,2)- abs(rob_size*cos(grad-pi/2))):m*stepsize:round(map(i+1,2)+abs(rob_size*cos(grad-pi/2)))]);
                 
            else
                container(2,:) =  round([round(map(i,2)+abs(rob_size*cos(grad-pi/2))):-m*stepsize:round(map(i+1,2)-abs(rob_size*cos(grad-pi/2)))]);
            end
            difference = diff(container(2,:));
            r = find(difference~=0);
            filler = [container(1,r) container(1,r)+ ones(1,length(r)); container(2,r)+ difference(r(1,1)).*ones(1,length(r)) container(2,r)];
            container = [container filler];
            
            
            
        else
            if map(i+1,2) > map(i,2)
                container(2,:) =  round(map(i,2)- rob_size:stepsize:round(map(i+1,2) + rob_size));
            else
                container(2,:) =  round(map(i,2)+ rob_size:-stepsize:round(map(i+1,2) - rob_size));
             end
            if map(i+1,1) > map(i,1)
            
               % container(1,:) = round([(map(i,1)- abs(rob_size*cos(grad-pi/2))):stepsize/m:(map(i+1,1)+abs(rob_size*cos(grad-pi/2)))]);
                container(1,:) = round((container(2,:)- map(i,2) + m*map(i,1))/m);
                 
            else
                container(1,:) =  round([(map(i,1)+abs(rob_size*cos(grad-pi/2))):-stepsize/m:(map(i+1,1)-abs(rob_size*cos(grad-pi/2)))]);
            end
            
%              if map(i+1,1) > map(i,1)
%                 container(1,:) =  round(map(i,1)- abs(rob_size*cos(grad-pi/2))):stepsize:round(map(i+1,1) + abs(rob_size*cos(grad-pi/2)));
%             else
%                 container(1,:) =  round(map(i,1)+ abs(rob_size*cos(grad-pi/2))):-stepsize:round(map(i+1,1) - abs(rob_size*cos(grad-pi/2)));
%              end
%             if map(i+1,2) > map(i,2)
%             
%                 container(2,:) = round([ round(map(i,2)- abs(rob_size*sin(grad-pi/2))):stepsize/m:round(map(i+1,2)+abs(rob_size*sin(grad-pi/2)))]);
%                  
%             else
%                 container(2,:) =  round([round(map(i,2)+abs(rob_size*sin(grad-pi/2))):-stepsize/m:round(map(i+1,2)-abs(rob_size*sin(grad-pi/2)))]);
%             end
            
            difference = diff(container(1,:));
            r = find(difference~=0);
            filler = [container(1,r)+ difference(r(1,1)).*ones(1,length(r)) container(1,r); container(2,r) container(2,r)+ones(1,length(r))];
            container = [container filler];
        end
%             
%         if map(i+1,1) > map(i,1)
%             container(1,:) =  round(map(i,1)- abs(rob_size*sin(grad-pi/2))):round(abs(stepsize*sin(grad-pi/2))):round(map(i+1,1) + abs(rob_size*sin(grad-pi/2)));
%         else
%             container(1,:) =  round(map(i,1) + abs(rob_size*sin(grad-pi/2))):round(-abs(stepsize*sin(grad-pi/2))):round(map(i+1,1)- abs(rob_size*sin(grad-pi/2)));
%         end
%         if map(i+1,2) > map(i,2)
%             i
%             container(2,:) =  round(map(i,2)- abs(rob_size*cos(grad-pi/2))):round(abs(stepsize*cos(grad-pi/2))):round(map(i+1,2)+abs(rob_size*cos(grad-pi/2)));
%         elseif map(i+1,2) < map(i,2)
%             container(2,:) =  round(map(i,2)+abs(rob_size*cos(grad-pi/2))):round(-abs(stepsize*cos(grad-pi/2))):round(map(i+1,2)-abs(rob_size*cos(grad-pi/2)));
%         else
%             container(2,:) =  map(i,2);
%         end
        if(rob_size>0)%to fix this
            if dy>0
                for j=-rob_size/stepsize:rob_size/stepsize
                    container_1(1,:) = (container (1,:) + j*stepsize);
                    container_1(2,:)= container(2,:);
                    container_2 = [ container_2 ; container_1'];
                end
            else
                 for j=-rob_size/stepsize:rob_size/stepsize
                    container_1(2,:) = (container (2,:) + j*stepsize);
                    container_1(1,:)= container(1,:);
                    container_2 = [ container_2 ; container_1'];
                 end
                 
             end
         else
             container_2 = container';
         end
        
   end
    closed = [ closed ; container_2];
    container =[];
    container_1=[];
    container_2 = [];
    filler =[];
    difference =[];
    r=[];


end

closed = unique(closed,'rows', 'stable');

%plot(closed(:,1),closed(:,2), 'rx');
end


