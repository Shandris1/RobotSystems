function adjacency = visbil_point_reduced(map,target,adjacency,dist, real_map)


%what happens in case of a tie for angles

%inclde code that checks if point is row vector or column vector
%implement to row



%http://www.david-gouveia.com/portfolio/pathfinding-on-a-2d-polygonal-map/
%https://books.google.co.uk/books?id=-V1QGo8KzzoC&pg=PA138&lpg=PA138&dq=visibility+graph+in+simple+polygon+lee&source=bl&ots=-hS27X9KW4&sig=H1mfkCFP7i7yyf4u7e4ndLqKUe0&hl=en&sa=X&ei=j6n8VI6qD8neU7XhgvAD&ved=0CDkQ6AEwBA#v=onepage&q=visibility%20graph%20in%20simple%20polygon%20lee&f=false
vertices = [adjacency(end-1:end,:); 1:size(adjacency,2)];

mapmod = [map; map(1,:)];
vertices_map = [map(1:end,:) [1:size(map,1)]']';

%check if its inside or not
[in on] = inpolygon(target(1),target(2),mapmod(:,1),mapmod(:,2));
edges = zeros(5,size(map,1));
for e = 1: size(mapmod,1) - 1
edges(:,e) = [ mapmod(e,:)'; mapmod(e+1,:)'; e];
end

%psuedo code
%compute angles to the horizontal and sort them also according to x axis in case
%for ties closest x axis should be put first
%store edges that intersect with the horizontal
%you have to populate ossible occlusion list
adjacency = [inf(size(adjacency,1),1) adjacency ];
adjacency = [inf(1,size(adjacency,2));adjacency];
adjacency(1,1) = 0;
%indices = [1: size(vertices,1);1: size(vertices,1)];

if(in&~on)%same procedure as before
%you can construct an adjacency mtrix to show connections

    %set current vertex at origin to calculate the angles
    
    
    %angles = delta_angle(i,vert_temp,edges)
   
    
    %kirsty
    % vert_temp = [vert_temp(:,1:2) - repmat(target,size(vert_temp,1),1) vert_temp(:,3)];
    %angles = sortrows([(mod((atan2(vert_temp(:,2),vert_temp(:,1))+2*pi),2*pi)) vert_temp(:,3)],1);
    
   % horizontal = repmat(vertices(i,[1 2]),1,2) + horizontal_delta;
   %horizontal = edges(i,1:4);
    %
    %S=[];
    %S = intersection_edge(repmat(horizontal,size(edges,1),1),edges);
    S = edges;
    %remove s that have ua = 0;
   
    %index_rem = find(S(:,6) == 0);
   %S(index_rem,:) = [];
    
    
   
   for j = 1:size(vertices,2)%%instead you should just go to the angle of previous vertex
%     j = 1;
%     flag = i;
%     %while(flag ~= stopping_ind)
%         if( angles(j,2) == stopping_ind)
%             flag = stopping_ind;
%         end
        
        
        pw = [ target(1); target(2); vertices(1:2,j)];%line of sight
        temp = intersection_edge(repmat(pw,1,size(S,2)),S);
        if( isempty(temp))
            testpoint = 1;
        end
        index_rem = find(roundn(temp(6,:),-5) == 0);
        if(~isempty(index_rem))
            temp(index_rem,:) = [];
        end
        %visibile = find(((temp(:,end)==0)||(temp(:,end)==1))==0);
        
        % it can be visbile either because its an edge or if it distance to
        % intersection is greater than the ray magnitude
        
        visibile_edge = (roundn(temp(end,1), -5)==0)||(roundn(temp(end,1), -5)==1);%its on an edge
        if(visibile_edge)%and looking at the right edge
            visibile = (visibile_edge) &&(roundn(temp(6,1),-5)== roundn(sqrt(sum((vertices(1:2,j)- [target(1); target(2)]).^2)),-5));
        else%not an edge but still visbile if intersection is behind it
            visibile = (temp(6,1)> sqrt(sum((vertices(1:2,j)- [target(1); target(2)]).^2)));
        end
            
        %if (isempty(visibile))
        if ((visibile))
            adjacency(1,j+1) = sqrt(sum(([target(1) target(2)]'-vertices(1:2,j)).^2));
            adjacency(j+1,1) =  adjacency(1,j+1);
        end

        temp = [];
        index_rem =[];
        
    
%http://www.dma.fi.upm.es/docencia/trabajosfindecarrera/programas/geometriacomputacional/Shortest_Path/html/E_Grafo_Visibilidad.htm

   end
   adjacency(end-1:end,1) = [target(1) target(2)]';
elseif(in&on)%on boundary
    % construct circle with 1.5*radius of reduction
    radius = 0.3;
    x = target(1)-radius : radius*2/500: target(1) +radius; 
   
    y = [target(2) + sqrt( (radius)^2 - (x'- target(1)).^2);target(2) - sqrt( (radius)^2 - (x(end:-1:1)'- target(1)).^2)];
    x = [x'; x(end:-1:1)'];
    %if you use median the vecor should be odd
    [in on] = inpolygon(x,y,mapmod(:,1),mapmod(:,2));
    new_locations = [x(in & ~on) y(in & ~on)];
    new_point = new_locations(round(size(new_locations,1)/2),:);
    target = real(new_point');
    S = edges;
    %remove s that have ua = 0;
   
    %index_rem = find(S(:,6) == 0);
   %S(index_rem,:) = [];
    
    
   
    for j = 1:size(vertices,2)%%instead you should just go to the angle of previous vertex
%     j = 1;
%     flag = i;
%     %while(flag ~= stopping_ind)
%         if( angles(j,2) == stopping_ind)
%             flag = stopping_ind;
%         end
        
        
        pw = [ target(1); target(2); vertices(1:2,j)];%line of sight
        temp = intersection_edge(repmat(pw,1,size(S,2)),S);
        index_rem = find(roundn(temp(6,:),-5) == 0);
        if(~isempty(index_rem))
            temp(index_rem,:) = [];
        end
        %visibile = find(((temp(:,end)==0)||(temp(:,end)==1))==0);
        
        % it can be visbile either because its an edge or if it distance to
        % intersection is greater than the ray magnitude
        
        visibile_edge = (roundn(temp(end,1), -5)==0)||(roundn(temp(end,1), -5)==1);%its on an edge
        if(visibile_edge)%and looking at the right edge
            visibile = (visibile_edge) &&(roundn(temp(6,1),-5)== roundn(sqrt(sum((vertices(1:2,j)- [target(1); target(2)]).^2)),-5));
        else%not an edge but still visbile if intersection is behind it
            visibile = (temp(6,1)> sqrt(sum((vertices(1:2,j)- [target(1); target(2)]).^2)));
        end
            
        %if (isempty(visibile))
        if ((visibile))
            adjacency(1,j+1) = sqrt(sum((target-vertices(1:2,j)).^2));
            adjacency(j+1,1) =  adjacency(1,j+1);
        end

        temp = [];
        index_rem =[];
        
    
%http://www.dma.fi.upm.es/docencia/trabajosfindecarrera/programas/geometriacomputacional/Shortest_Path/html/E_Grafo_Visibilidad.htm

    end
    adjacency(end-1:end,1) = [target(1) target(2)]';
%http://www.dma.fi.upm.es/docencia/trabajosfindecarrera/programas/geometriacomputacional/Shortest_Path/html/E_Grafo_Visibilidad.htm

    
else %outside of map
    
    % construct circle with 1.5*radius of reduction
    mapmod_real = [real_map; real_map(1,:)];
    edges_real = zeros(5,size(real_map,1));
    for e = 1: size(mapmod_real,1) - 1
    edges_real(:,e) = [ mapmod_real(e,:)'; mapmod_real(e+1,:)'; e];
    end
    
    
    
    for loop = 1: size(edges_real,2)
        point_to_line = [target(1); target(2)] - edges_real(1:2, loop);
    %edge
        line_vec = edges_real(3:4, loop) - edges_real(1:2, loop);
    
        dot1 = sum(point_to_line.*line_vec,1);
        dot2 = sum(line_vec.*line_vec,1);
    
        if ( dot1  <= 0 )
            direction_vector_real(:,loop) = [  edges_real(1:4, loop);sqrt(sum(([target(1) target(2)]'-edges_real(1:2, loop)).^2)) ];
           
               
        elseif( dot2 <= dot1 ) 
         
                
            
        direction_vector_real(:,loop) = [ edges_real(1:4, loop);sqrt(sum(([target(1) target(2)]'-edges_real(3:4, loop)).^2)) ];
            
                
        else

            distance_parallel = dot1 / dot2;
            boundary_point = edges_real(1:2, loop) + distance_parallel.*line_vec;
            direction_vector_real(:,loop) = [ edges_real(1:4, loop) ;sqrt(sum(([target(1) target(2)]'-boundary_point)).^2) ];
       end
    end
    
    
    
%     x = target(1)-1.25*dist : 2.5*dist/50: target(1) +1.25*dist; 
%    
%     y = [target(2) + sqrt( (1.25*dist)^2 - (x'- target(1)).^2);target(2) - sqrt( (1.25*dist)^2 - (x(end:-1:1)'- target(1)).^2)];
%     x = [x'; x(end:-1:1)'];
%     %if you use median the vecor should be odd
%     [in on] = inpolygon(x,y,mapmod(:,1),mapmod(:,2));
%     new_locations = [x(in & ~on) y(in & ~on)];
%     new_point = new_locations(round(size(new_locations,1)/2),:);
%     
    
    
    edge_block = sortrows(direction_vector_real',5);
    S = [edges [edge_block(1,1:4) size(edges,2)+1]' [edge_block(2,1:4) size(edges,2)+2]'];
    possible_vert = [];
    counter =1;
    
        for j = 1:length(vertices_map)%%instead you should just go to the angle of previous vertex
%     j = 1;
%     flag = i;
%     %while(flag ~= stopping_ind)
%         if( angles(j,2) == stopping_ind)
%             flag = stopping_ind;
%         end
        
            pw = [ target(1); target(2); vertices_map(1:2,j)];%line of sight
            temp = intersection_edge(repmat(pw,1,size(S,2)),S);
            index_rem = find(roundn(temp(6,:),-5) == 0);%assumes no vertices on the same line
            if(~isempty(index_rem))
                temp(index_rem,:) = [];
            end
        %visibile = find(((temp(:,end)==0)||(temp(:,end)==1))==0);
        
        % it can be visbile either because its an edge or if it distance to
        % intersection is greater than the ray magnitude
        
            visibile_edge = (roundn(temp(end,1), -5)==0)||(roundn(temp(end,1), -5)==1);%its on an edge
            if(visibile_edge)%and looking at the right edge
                visibile = (visibile_edge) &&(roundn(temp(6,1),-5)== roundn(sqrt(sum((vertices_map(1:2,j)- [target(1); target(2)]).^2)),-5));
            else%not an edge but still visbile if intersection is behind it
                visibile = (temp(6,1)> sqrt(sum((vertices_map(1:2,j)- [target(1); target(2)]).^2)));
            end
       
        %if (isempty(visibile))
            if ((visibile))
                possible_vert(:,counter) = [vertices_map(3,j) sqrt(sum(([target(1) target(2)]'-vertices_map(1:2,j)).^2))];
                counter = counter +1 ;
            end
        end
    
    %sort it by distance
        possible_vert = sortrows(possible_vert',2);
%         difference_array = abs(diff(possible_vert(:,1)));
%         difference_array(difference_array == abs(1-size(vertices_map,2))) = 1;
%         difference_array_index = possible_vert(find(roundn(difference_array,-5)==1,1, 'first'),1);
        difference_array_index = possible_vert(1,1);
    %check both edges that touch with this vertex and find the smallest
    %distance and choose that
    % vector from point to line for case 1 vertexi to vertex i+1
        point_to_line = [target(1); target(2)] - edges(1:2, difference_array_index);
    %edge
        line_vec = edges(3:4, difference_array_index) - edges(1:2, difference_array_index);
    
        dot1 = sum(point_to_line.*line_vec,1);
        dot2 = sum(line_vec.*line_vec,1);
    
        if ( dot1  <= 0 )
            direction_vector(:,1) = [ target(1); target(2); edges(1:2, difference_array_index);sqrt(sum(([target(1) target(2)]'-edges(1:2, difference_array_index)).^2)) ];
           
               
        elseif( dot2 <= dot1 ) 
         
                
            if(difference_array_index == size(edges,2))
                direction_vector(:,1) = [ target(1); target(2); edges(1:2, 1);sqrt(sum(([target(1) target(2)]'-edges(1:2, 1)).^2)) ];
            else
                direction_vector(:,1) = [ target(1); target(2); edges(1:2, difference_array_index + 1);sqrt(sum(([target(1) target(2)]'-edges(1:2, difference_array_index + 1)).^2)) ];
            end
                
        else

            distance_parallel = dot1 / dot2;
            boundary_point = edges(1:2, difference_array_index) + distance_parallel.*line_vec;
            direction_vector(:,1) = [ target(1); target(2); boundary_point ;sqrt(sum(([target(1) target(2)]'-boundary_point)).^2) ];
       end

     % vector from point to line for case 1 vertexi to vertex i-1
       if(difference_array_index == 1)
            point_to_line = [target(1); target(2)] - edges(1:2, size(edges,2));
    %edge
            line_vec = edges(3:4, size(edges,2)) - edges(1:2, size(edges,2));
    
    
    
            dot1 = sum(point_to_line.*line_vec,1);
            dot2 = sum(line_vec.*line_vec,1);
    
            if ( dot1  <= 0 )
                direction_vector(:,2) = [ target(1); target(2); edges(1:2, size(edges,2));sqrt(sum(([target(1) target(2)]'-edges(1:2, size(edges,2))).^2)) ];
           
               
            elseif( dot2 <= dot1 ) 
                
                direction_vector(:,2) = [ target(1); target(2); edges(1:2, difference_array_index);sqrt(sum(([target(1) target(2)]'-edges(1:2, difference_array_index )).^2)) ];
                
                
            else

                distance_parallel = dot1 / dot2;
                boundary_point = edges(1:2, size(edges,2)) + distance_parallel.*line_vec;
                direction_vector(:,2) = [ target(1); target(2); boundary_point ;sqrt(sum(([target(1) target(2)]'-boundary_point)).^2) ];
            end
       else
        
            point_to_line = [target(1); target(2)] - edges(1:2, difference_array_index -1 );
    %edge
            line_vec = edges(3:4, difference_array_index-1) - edges(1:2, difference_array_index -1);
    
            dot1 = sum(point_to_line.*line_vec,1);
            dot2 = sum(line_vec.*line_vec,1);
    
            if ( dot1  <= 0 )
                direction_vector(:,2) = [ target(1); target(2); edges(1:2, difference_array_index-1);sqrt(sum(([target(1) target(2)]'-edges(1:2, difference_array_index-1)).^2)) ];
           
               
            elseif( dot2 <= dot1 ) 
                
                
                direction_vector(:,2) = [ target(1); target(2); edges(1:2, difference_array_index );sqrt(sum(([target(1) target(2)]'-edges(1:2, difference_array_index )).^2)) ];
              
                
            else

                distance_parallel = dot1 / dot2;
                boundary_point = edges(1:2, difference_array_index -1) + distance_parallel.*line_vec;
                direction_vector(:,2) = [ target(1); target(2); boundary_point ;sqrt(sum(([target(1) target(2)]'-boundary_point)).^2) ];
            end  
        
        
        end
    
    %function to compute distance from point to line
    
    
    
    
    
    
    %%achieve new point
      new_location = sortrows(direction_vector',5);
      %vector_final  = (new_location(1,3:4) - new_location(1,1:2));
%      vector_final = vector_final./(abs(vector_final(1) + 1i*vector_final(2)));
      
      %new_point = repmat(new_location(1,3:4),11,1) + (0:0.1:1)'*(vector_final);
      radius = 0.3;
      x = new_location(1,3)-radius : radius*2/500: new_location(1,3) +radius; 
      y = [new_location(1,4) + sqrt( (radius)^2 - (x'- new_location(1,3)).^2);new_location(1,4) - sqrt( (radius)^2 - (x(end:-1:1)'- new_location(1,3)).^2)];
      x = [x'; x(end:-1:1)'];
      [in on] = inpolygon(x,y,mapmod(:,1),mapmod(:,2));
      new_positions = [x(in & ~on) y(in & ~on)];
      new_point = new_positions(round(size(new_positions,1)/2),:);
      target = real(new_point');
%       new_target = [new_pointx(repmat(in & ~on),1,size(new_point,2))];
%       target = new_target(end-1:end);
        S=[];
      S = edges;
    %remove s that have ua = 0;
   
    %index_rem = find(S(:,6) == 0);
   %S(index_rem,:) = [];
    
    
   
     for j = 1:size(vertices,2)%%instead you should just go to the angle of previous vertex
%     j = 1;
%     flag = i;
%     %while(flag ~= stopping_ind)
%         if( angles(j,2) == stopping_ind)
%             flag = stopping_ind;
%         end
        
        
        pw = [ target(1); target(2); vertices(1:2,j)];%line of sight
        temp = intersection_edge(repmat(pw,1,size(S,2)),S);
        if( isempty(temp))
            testpoint = 1;
        end
        index_rem = find(roundn(temp(6,:),-5) == 0);
        
        if(~isempty(index_rem))
            temp(index_rem,:) = [];
        end
        %visibile = find(((temp(:,end)==0)||(temp(:,end)==1))==0);
        
        % it can be visbile either because its an edge or if it distance to
        % intersection is greater than the ray magnitude
        
        visibile_edge = (roundn(temp(end,1), -5)==0)||(roundn(temp(end,1), -5)==1);%its on an edge
        if(visibile_edge)%and looking at the right edge
            visibile = (visibile_edge) &&(roundn(temp(6,1),-5)== roundn(sqrt(sum((vertices(1:2,j)- [target(1); target(2)]).^2)),-5));
        else%not an edge but still visbile if intersection is behind it
            visibile = (temp(6,1)> sqrt(sum((vertices(1:2,j)- [target(1); target(2)]).^2)));
        end
            
        %if (isempty(visibile))
        if ((visibile))
            adjacency(1,j+1) = sqrt(sum((target-vertices(1:2,j)).^2));
            adjacency(j+1,1) =  adjacency(1,j+1);
        end

        temp = [];
     index_rem =[];
        
    
%http://www.dma.fi.upm.es/docencia/trabajosfindecarrera/programas/geometriacomputacional/Shortest_Path/html/E_Grafo_Visibilidad.htm

    end
   adjacency(end-1:end,1) = [target(1) target(2)]';
%http://www.dma.fi.upm.es/docencia/trabajosfindecarrera/programas/geometriacomputacional/Shortest_Path/html/E_Grafo_Visibilidad.htm

    
    
    

end
   
end



  function edge_sort = intersection_edge(edge1, edge2)
    %http://paulbourke.net/geometry/pointlineplane/
    x43 = edge2(3,:)-edge2(1,:);
    y43=  edge2(4,:)-edge2(2,:);
    x21 = edge1(3,:)-edge1(1,:);
    y21 = edge1(4,:)-edge1(2,:);
    x13 = edge1(1,:) - edge2(1,:);
    y13 = edge1(2,:) - edge2(2,:);
    
     
    ua = (x43.*y13-y43.*x13)./(y43.*x21-x43.*y21);
    ub = (x21.*y13-y21.*x13)./(y43.*x21-x43.*y21);
    
    %intersection_index = find((ub >= 0 & ub <= 1 & ua>=0)>0);
    intersection_index = ((roundn(ub,-5) >= 0 & roundn(ub,-5) <= 1 & roundn(ua,-5)>=0)>0);
    %if(~isempty(intersection_index))
     if(sum(intersection_index)~= 0)
   % edge_sort = edge2(1:5, intersection_index);
    edge_sort = edge2(repmat(intersection_index,5,1));
    edge_sort = reshape(edge_sort,[5 length(edge_sort)/5]);
    edge_sort = [ edge_sort ; abs(x21(intersection_index) + 1*1i*y21(intersection_index)).*ua(intersection_index); ub(intersection_index)];
    edge_sort = (sortrows(edge_sort',6))';
    else
        edge_sort = [];
    end
    
    
    end
    
    function angle = delta_angle(vertex1_no,vertices,edges)
    vec1 = edges(vertex1_no,3:4)-edges(vertex1_no,1:2);
    vec1 = repmat(vec1,size(vertices,1),1);
    vectors = vertices(:,1:2) -repmat(edges(vertex1_no,1:2),size(vertices,1),1);
    dot = sum(vec1.*vectors,2);
    cross =vec1(:,1).*vectors(:,2)-vec1(:,2).*vectors(:,1);
    angle = sortrows([(mod((atan2(cross,dot)+2*pi),2*pi)) vertices(:,3)],1);
    end
    
    
    
    

 
    
    