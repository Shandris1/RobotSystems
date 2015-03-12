function [children] = succ_vis( node_no, adjacency_new,gvalue, target,robot)


%child vector
%x y xparent y parent h g f
%check all 8 neighbours
%to be consistent h(x1)-h(x2)<= g(x1,x2)
%cost function can be done in mm for diag it will be 15 as sqrt(200) =
%14.14 approx to 15 and up down left right = 10
count = 0;
children =[];



       for i =1 : size(adjacency_new,2)
           if(~isinf(adjacency_new(node_no,i)))
                count = count +1;
%                   xpos =child_x + offsetx -1;
%                   ypos = rangey-offsety-child_y;
                  h = floor(abs(adjacency_new(end-1,i)+ 1i*adjacency_new(end,i) - target(1) + 1i*target(2)));
                  %h = abs(xpos+ 1i*ypos - (target(1) + 1i*target(2)));
                  g = gvalue + ceil(adjacency_new(node_no,i));
                  
                  %g = parents (1,3) + (abs((parents(1) + 1i*parents(2))-(xpos+ 1i*ypos)));
                  children(count,:) = [i node_no h g h+g];
           end
           
       end
        
            
         
        
 end          
