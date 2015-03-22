function [reduced, flag_cw]= reduce_graph_robot(map,dist)



%http://www.david-gouveia.com/portfolio/pathfinding-on-a-2d-polygonal-map/
%https://books.google.co.uk/books?id=-V1QGo8KzzoC&pg=PA138&lpg=PA138&dq=visibility+graph+in+simple+polygon+lee&source=bl&ots=-hS27X9KW4&sig=H1mfkCFP7i7yyf4u7e4ndLqKUe0&hl=en&sa=X&ei=j6n8VI6qD8neU7XhgvAD&ved=0CDkQ6AEwBA#v=onepage&q=visibility%20graph%20in%20simple%20polygon%20lee&f=false
vertices = [map(1:end,:) [1:size(map,1)]'];
reduced = zeros(length(vertices),2);
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
vector_edges = (edges(:,3:4)- edges(:,1:2));
vector_edges = vector_edges./repmat((abs(vector_edges(:,1) + 1*j*vector_edges(:,2))),1,2);


%you can construct an adjacency mtrix to show connections
%check if its clockwise or anticlockwise defined
%find leftmost vertex
leftmost = sortrows(vertices,[ 2 1]);% is this the best way to order it are we sure??
leftmost_ind = leftmost(1,3);
%perform cross product between previous edge and current edge
prev_edge = leftmost_ind-1;
    if (prev_edge == 0)
        prev_edge = size(vertices,1);
    end
angle = angle_vec(edges(prev_edge,3:4)-edges(prev_edge,1:2),edges(leftmost_ind,:));
if(angle(1,1) > pi)
    flag_cw =1;
else
    flag_cw =0;
end
%vertices = circshift(vertices,-(leftmost_ind-1));
%cw x+dist y + dist
counter = 1;

    for i = 1: length(vertices)
        if i ==1
        vector =  (vector_edges(1,:))-(vector_edges(end,:));
        
        else
        vector =  (vector_edges(i,:))-(vector_edges(i-1,:));
        
        end
        normalisation = (abs(vector(1) + 1i*vector(2)));
        if (normalisation ~= 0)
         vector = vector/normalisation;
        %check orientation if not good reverse
        
        check_ang = angle_vec (vector,edges(i,:));
        if (flag_cw)&&(check_ang(1,1) < pi)            
        reduced(i,:) = vertices(i,1:2) + abs(dist/sin(check_ang(1,1)))*vector;
      
        elseif (flag_cw)
         
         
          reduced(i,:) = vertices(i,1:2) - abs(dist/sin(check_ang(1,1)))*vector ;
          
          
          %create two vertices using minus vectors times
         %dist/cos(heta)*(1/sin(theta -1)
        elseif(flag_cw==0)&&(check_ang(1,1) > pi) 
         reduced(i,:) = vertices(i,1:2) + abs(dist/sin(check_ang(1,1)))*vector;
          
        else
            
        
          
          reduced(i,:) = vertices(i,1:2) - abs(dist/sin(check_ang(1,1)))*vector ;
          
            
%         reduced(counter,:) = vertices(i,1:2) - abs(dist/sin(check_ang(1,1)))*vector;
%          counter = counter +1;
        end
        end
         
    end
    
% %check if it is self intersecting
% % construct matrix with all edges repeated edge 1 edge2......edge 1 edg2
% % and a matrix which is edge1 edge1....edgen edge n
% edges_red = zeros(4,size(reduced,1));
% mapmod_red = [reduced; reduced(1,:)];
% for e = 1: size(reduced,1) - 1
% edges_red(:,e) = [ mapmod_red(e,:)'; mapmod_red(e+1,:)'];
% end
% edge2 = repmat(edges_red,1,size(reduced,1));
% 
% edge1 = reshape((repmat(edge2(:,1:size(reduced,1)),size(reduced,1),1)),[4 size(reduced,1)^2]);
% 
% %perform intersection test
%  
%     %http://paulbourke.net/geometry/pointlineplane/
%     x43 = edge2(3,:)-edge2(1,:);
%     y43=  edge2(4,:)-edge2(2,:);
%     x21 = edge1(3,:)-edge1(1,:);
%     y21 = edge1(4,:)-edge1(2,:);
%     x13 = edge1(1,:) - edge2(1,:);
%     y13 = edge1(2,:) - edge2(2,:);
%     
%      
%     ua = (x43.*y13-y43.*x13)./(y43.*x21-x43.*y21);
%     ub = (x21.*y13-y21.*x13)./(y43.*x21-x43.*y21);
%     
%     %intersection_index = find((ub >= 0 & ub <= 1 & ua>=0)>0);
%      intersection_index = ((roundn(ub,-5) >= 0 & roundn(ub,-5) <= 1 & roundn(ua,-5)>=0)>0);
%     %if(~isempty(intersection_index))
%      if(sum(intersection_index)~= 0)
%    % edge_sort = edge2(1:5, intersection_index);
%     edge_sort = edge2(repmat(intersection_index,5,1));
%     edge_sort = reshape(edge_sort,[5 length(edge_sort)/5]);
%     edge_sort = [ edge_sort ; abs(x21(intersection_index) + 1*1i*y21(intersection_index)).*ua(intersection_index); ub(intersection_index)];
%     edge_sort = (sortrows(edge_sort',6))';
%     else
%         edge_sort = [];
%     end
%     
    
    

end
 function angle = angle_vec(vec1,vec2)
    vectors = vec2(:,3:4)- vec2(:,1:2);
    dot = sum(vec1.*vectors,2);
    cross =vec1(:,1).*vectors(:,2)-vec1(:,2).*vectors(:,1);
    angle = sortrows([(mod((atan2(cross,dot)+2*pi),2*pi)) vec2(:,5)],1);
    end