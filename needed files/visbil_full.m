function adjacency = visbil_full(map,botsim,target, flag_cw)


%what happens in case of a tie for angles


% use always vectors tha are in columns


%http://www.david-gouveia.com/portfolio/pathfinding-on-a-2d-polygonal-map/
%https://books.google.co.uk/books?id=-V1QGo8KzzoC&pg=PA138&lpg=PA138&dq=visibility+graph+in+simple+polygon+lee&source=bl&ots=-hS27X9KW4&sig=H1mfkCFP7i7yyf4u7e4ndLqKUe0&hl=en&sa=X&ei=j6n8VI6qD8neU7XhgvAD&ved=0CDkQ6AEwBA#v=onepage&q=visibility%20graph%20in%20simple%20polygon%20lee&f=false
vertices = [map(1:end,:) [1:size(map,1)]']';
horizontal_delta = [ 0 0 1 0];
mapmod = [map; map(1,:)];
edges = zeros(5,size(map,1));
for e = 1: size(mapmod,1) - 1
edges(:,e) = [ mapmod(e,:)'; mapmod(e+1,:)'; e];
end
%edges = [edges'; [1:size(map,1)]];
%psuedo code
%compute angles to the horizontal and sort them also according to x axis in case
%for ties closest x axis should be put first
%store edges that intersect with the horizontal
%you have to populate ossible occlusion list
adjacency = inf(size(vertices,2));
%indices = [1: size(vertices,1);1: size(vertices,1)];
new_ind = sub2ind(size(adjacency), 1: size(vertices,1),1: size(vertices,1));
adjacency(new_ind) = 0;



    

for i = 1:size(vertices,2)
    %set current vertex at origin to calculate the angles
    vert_temp = vertices;
    vert_temp(:,i) = [];
    angles = delta_angle(i,vert_temp,edges,flag_cw);
   % vert_temp = [vert_temp(:,1:2) - repmat(vertices(i,1:2),size(vert_temp,1),1) vert_temp(:,3)];
    %angles = sortrows([(mod((atan2(vert_temp(:,2),vert_temp(:,1))+2*pi),2*pi)) vert_temp(:,3)],1);
    
  
 
    S = edges;
   
    
    
    stopping_ind = i-1;
    if (stopping_ind == 0)
        stopping_ind = size(vertices,2);
    end
    %for j = 1:length(angles)%%instead you should just go to the angle of previous vertex
    j = 1;
    flag = i;
    while(flag ~= stopping_ind)
        if( angles(2,j) == stopping_ind)
            flag = stopping_ind;
        end
        
        
        pw = [ vertices(1:2,i); vertices(1:2,angles(2,j))];%line of sight
        temp = intersection_edge(repmat(pw,1,size(S,2)),S);
         index_rem = find(temp(6,:) == 0);
        temp(:,index_rem) = [];
        %visibile = find(((temp(:,end)==0)||(temp(:,end)==1))==0);
        visibile = (roundn(temp(end,1),-5)==0)||(roundn(temp(end,1),-5)==1);%its on an edge
        visibile = (visibile) &&(roundn(temp(6,1),-5)== roundn(sqrt(sum((vertices(1:2,angles(2,j))- vertices(1:2,i)).^2)),-5));
        %if (isempty(visibile))
        if ((visibile)||(flag==stopping_ind))
            adjacency(i,angles(2,j)) = sqrt(sum((vertices(1:2,i)-vertices(1:2,angles(2,j))).^2));
        end
        % the current vertex we are pointing towrda is the start of an edge
        % we include that edge in S, if it is the end of an edge we remove
        % it
        

       
        j = j+1;
        temp = [];
       
        
    
%http://www.dma.fi.upm.es/docencia/trabajosfindecarrera/programas/geometriacomputacional/Shortest_Path/html/E_Grafo_Visibilidad.htm

    end
end
adjacency = [adjacency ;vertices(1:2,:)];
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
    
    function angle = delta_angle(vertex1_no,vertices,edges, flag)
    if(flag)
    vec1 = edges(3:4,vertex1_no)-edges(1:2,vertex1_no);
    vec1 = repmat(vec1,1,size(vertices,2));
    vectors = vertices(1:2,:) -repmat(edges(1:2,vertex1_no),1,size(vertices,2));
    dot = sum(vec1.*vectors,1);
    cross =vec1(1,:).*vectors(2,:)-vec1(2,:).*vectors(1,:);
    angle_temp = [sortrows([(mod((atan2(cross,dot)+2*pi),2*pi))' vertices(3,:)'],-1)]';
%     angle_zero_ind = find(angle_temp(:,1)==0);
%     angle = [angle_temp(angle_zero_ind,:)];
%     angle_temp(angle_zero_ind,:) = [];
%     angle = [angle; angle_temp];
    
    angle_zero_ind =  repmat(angle_temp(1,:)==0,2,1);
    angle = [angle_temp(angle_zero_ind)];
    angle = reshape(angle, [2 length(angle)/2]);
    angle_temp(angle_zero_ind) = [];
    angle_temp = reshape(angle_temp, [2 length(angle_temp)/2]);
    angle = [angle angle_temp];
    
    else
    vec1 = edges(3:4,vertex1_no)-edges(1:2,vertex1_no);
    vec1 = repmat(vec1,1,size(vertices,2));
    vectors = vertices(1:2,:) -repmat(edges(1:2,vertex1_no),1,size(vertices,2));
    dot = sum(vec1.*vectors,1);
    cross =vec1(1,:).*vectors(2,:)-vec1(2,:).*vectors(1,:);
    angle = [sortrows([(mod((atan2(cross,dot)+2*pi),2*pi))' vertices(3,:)'],1)]';
    end
    
    end
    
    
    function angle = angle_vec(vec1,vec2)
    vectors = vec2(:,3:4)- vec2(:,1:2);
    dot = sum(vec1.*vectors,2);
    cross =vec1(:,1).*vectors(:,2)-vec1(:,2).*vectors(:,1);
    angle = sortrows([(mod((atan2(cross,dot)+2*pi),2*pi)) vec2(:,5)],1);
    end
    
 
    
    