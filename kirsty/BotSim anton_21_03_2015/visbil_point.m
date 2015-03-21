function adjacency = visbil_point(map,target,adjacency)


%what happens in case of a tie for angles

%inclde code that checks if point is row vector or column vector
%implement to row



%http://www.david-gouveia.com/portfolio/pathfinding-on-a-2d-polygonal-map/
%https://books.google.co.uk/books?id=-V1QGo8KzzoC&pg=PA138&lpg=PA138&dq=visibility+graph+in+simple+polygon+lee&source=bl&ots=-hS27X9KW4&sig=H1mfkCFP7i7yyf4u7e4ndLqKUe0&hl=en&sa=X&ei=j6n8VI6qD8neU7XhgvAD&ved=0CDkQ6AEwBA#v=onepage&q=visibility%20graph%20in%20simple%20polygon%20lee&f=false
vertices = [adjacency(end-1:end,:)' [1:size(adjacency,2)]'];
horizontal_delta = [ 0 0 1 0];
mapmod = [map; map(1,:)];
for e = 1: size(mapmod,1) - 1
edges(e,:) = [ mapmod(e,:) mapmod(e+1,:)];
end
edges = [edges [1:size(map,1)]'];
%psuedo code
%compute angles to the horizontal and sort them also according to x axis in case
%for ties closest x axis should be put first
%store edges that intersect with the horizontal
%you have to populate ossible occlusion list
adjacency = [inf(size(adjacency,1),1) adjacency ];
 adjacency = [inf(1,size(adjacency,2));adjacency];
 adjacency(1,1) = 0;
%indices = [1: size(vertices,1);1: size(vertices,1)];


%you can construct an adjacency mtrix to show connections

    %set current vertex at origin to calculate the angles
    vert_temp = vertices;
    
    %angles = delta_angle(i,vert_temp,edges)
    vert_temp = [vert_temp(:,1:2) - repmat(target,size(vert_temp,1),1) vert_temp(:,3)];
    angles = sortrows([(mod((atan2(vert_temp(:,2),vert_temp(:,1))+2*pi),2*pi)) vert_temp(:,3)],1);
    
   % horizontal = repmat(vertices(i,[1 2]),1,2) + horizontal_delta;
   %horizontal = edges(i,1:4);
    %
    %S=[];
    %S = intersection_edge(repmat(horizontal,size(edges,1),1),edges);
    S = edges;
    %remove s that have ua = 0;
   
    %index_rem = find(S(:,6) == 0);
   %S(index_rem,:) = [];
    
    
   
   for j = 1:length(angles)%%instead you should just go to the angle of previous vertex
%     j = 1;
%     flag = i;
%     %while(flag ~= stopping_ind)
%         if( angles(j,2) == stopping_ind)
%             flag = stopping_ind;
%         end
        
        
        pw = [ target vertices(angles(j,2),1:2)];%line of sight
        temp = intersection_edge(repmat(pw,size(S,1),1),S);
         index_rem = find(roundn(temp(:,6),-5) == 0);
         if(~isempty(index_rem))
         temp(index_rem,:) = [];
         end
        %visibile = find(((temp(:,end)==0)||(temp(:,end)==1))==0);
        
        % it can be visbile either because its an edge or if it distance to
        % intersection is greater than the ray magnitude
        
        visibile_edge = (roundn(temp(1,end), -5)==0)||(roundn(temp(1,end), -5)==1);%its on an edge
        if(visibile_edge)%and looking at the right edge
         visibile = (visibile_edge) &&(roundn(temp(1,6),-5)== roundn(sqrt(sum((vertices(angles(j,2),1:2)- target).^2)),-5));
        else%not an edge but still visbile if intersection is behind it
            visibile = (temp(1,6)> sqrt(sum((vertices(angles(j,2),1:2)- target).^2)));
        end
            
        %if (isempty(visibile))
        if ((visibile))
            adjacency(1,angles(j,2)+1) = sqrt(sum((target-vertices(angles(j,2),1:2)).^2));
            adjacency(angles(j,2)+1,1) =  adjacency(1,angles(j,2)+1);
        end
        % the current vertex we are pointing towrda is the start of an edge
        % we include that edge in S, if it is the end of an edge we remove
        % it
        
        %to debug this how to define end and start of edge....
%         prev = angles(j,2)-1;
%         if (prev == 0)
%             prev = size(vertices,1);
%         end
%             
%         %find edges to add
%         check_edges = angle_vec(repmat(pw(1,3:4)- pw(1,1:2),2,1),[edges(prev,3:4) edges(prev,1:2) edges(prev,5); edges(angles(j,2),:)]);
%         
%         if (check_edges(1,1)>0)&&(check_edges(1,1)<pi)
%             check_S_start = find(S(:,5) == check_edges(1,2));
%             
%             if ( isempty(check_S_start))
%             %index = find( temp(:,5)==angles(j,2));
%             new = intersection_edge(pw,edges(check_edges(1,2),:));
%             S = [S ; new];
%             end
%             check_S_start =[];
%         else
%             check_S_end = find(S(:,5) == check_edges(1,2));
%             if ( ~isempty(check_S_end))
%             index = find( S(:,5)==check_edges(1,2));
%             S(index,:) = [];
%             end
%             check_S_end =[];
%         end
%         if (check_edges(2,1)>0)&&(check_edges(2,1)<pi)
%             check_S_start = find(S(:,5) == check_edges(2,2));
%             if ( isempty(check_S_start))
%             %index = find( temp(:,5)==angles(j,2));
%             new = intersection_edge(pw,edges(check_edges(2,2),:));
%             S = [S ; new];
%             end
%             check_S_start =[];
%            
%         else
%             check_S_end = find(S(:,5) == check_edges(2,2));
%             if ( ~isempty(check_S_end))
%             index = find( S(:,5)==check_edges(2,2));
%             S(index,:) = [];
%             end
%             check_S_end =[];
%         end
%         
%             
%         % find edges to delete
%         %check_S_start = find(S(:,5) == angles(j,2));
%         %check_S_end = find(S(:,5) == (angles(j,2)-1));
% %         if ( isempty(check_S_start))
% %             %index = find( temp(:,5)==angles(j,2));
% %             new = intersection_edge(pw,edges(angles(j,2),:));
% %             S = [S ; new];
% %         end
% %         if ( ~isempty(check_S_end))
% %             index = find( S(:,5)==angles(j,2)-1);
% %             S(index,:) = [];
% %         end
%         S = sortrows(S,6)
%         
       
       % j = j+1;
        temp = [];
     index_rem =[];
        
    
%http://www.dma.fi.upm.es/docencia/trabajosfindecarrera/programas/geometriacomputacional/Shortest_Path/html/E_Grafo_Visibilidad.htm

   end
    adjacency(end-1:end,1) = target;
end



    function edge_sort = intersection_edge(edge1, edge2)
    
    x43 = edge2(:,3)-edge2(:,1);
    y43=  edge2(:,4)-edge2(:,2);
    x21 = edge1(:,3)-edge1(:,1);
    y21 = edge1(:,4)-edge1(:,2);
    x13 = edge1(:,1) - edge2(:,1);
    y13 = edge1(:,2) - edge2(:,2);
    
     
    ua = (x43.*y13-y43.*x13)./(y43.*x21-x43.*y21);
    ub = (x21.*y13-y21.*x13)./(y43.*x21-x43.*y21);
    
    intersection_index = find((ub >= 0 & ub <= 1 & ua>=0)>0);
    if(~isempty(intersection_index))
    edge_sort = edge2(intersection_index,1:5);
    edge_sort = [ edge_sort abs(x21(intersection_index) + 1*1i*y21(intersection_index)).*ua(intersection_index) ub(intersection_index)];
    edge_sort = sortrows(edge_sort,6);
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
    
    
    function angle = angle_vec(vec1,vec2)
    vectors = vec2(:,3:4)- vec2(:,1:2);
    dot = sum(vec1.*vectors,2);
    cross =vec1(:,1).*vectors(:,2)-vec1(:,2).*vectors(:,1);
    angle = sortrows([(mod((atan2(cross,dot)+2*pi),2*pi)) vec2(:,5)],1);
    end
    
 
    
    