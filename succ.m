function [children] = succ(parents,grid,offsetx,offsety, rangey,target,robot)

x_index = -offsetx+1+ parents(1);
y_index = rangey+offsety- parents(2);
%child vector
%x y xparent y parent h g f
%check all 8 neighbours
%to be consistent h(x1)-h(x2)<= g(x1,x2)
%cost function can be done in mm for diag it will be 15 as sqrt(200) =
%14.14 approx to 15 and up down left right = 10
count = 0;
children =[];
for j= -1:1:1
    for k = -1:1:1
        if~(k==0 && j==0)
            child_x = x_index + j;
            child_y = y_index + k;
       
        
            if(child_x>0)&&(child_x<size(grid,2))&&(child_y>0)&&(child_y<size(grid,1))%valid index
                  xpos =child_x + offsetx -1;
                  ypos = rangey+offsety-child_y;
                if(grid(child_y,child_x)~=-1)%&&(robot.pointInsideMap([xpos ypos]))%not in closed
                  count = count +1;
%                   xpos =child_x + offsetx -1;
%                   ypos = rangey-offsety-child_y;
                  h = floor(abs(xpos+ 1i*ypos - (target(1) + 1i*target(2))));
                  g = parents (1,3) + ceil(abs((parents(1) + 1i*parents(2))-(xpos+ 1i*ypos)));
                  children(count,:) = [xpos ypos parents(1) parents(2) h g h+g];
                end
             end
        end
        
    end          
end
%children = sortrows(children,7);
end