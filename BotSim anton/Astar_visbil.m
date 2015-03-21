%function [move_commands turn_commands optimal] = Astar_visbil(currentPos,currentAng,target,closed,stepsize,robot)
function [output_move output_angle optimal] = Astar_visbil(currentPos,currentAng, adjacency,map,robot,stepsize,target_real,dist,map_real)
%check that it is divisble by max range

%to do
%convert rotation facing target
%convert x y positions to commnds for simultation
%include different step sizes
target = adjacency(end-1:end,1);
%target is always node 2;
%start point is always node 1
adjacency_new = visbil_point_reduced(map,currentPos ,adjacency,dist,map_real);
temp_current =adjacency_new(end-1:end,1);

open = [];
start = 0;
final =0;
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


% range_x = max(closed(:,1))- min(closed(:,1)) + 1;
% offsetx = min(closed(:,1));
% range_y = max(closed(:,2))- min(closed(:,2)) + 1;
% offsety = min(closed(:,2));


%size_closed = size(closed,1);

size_closed = 0;
% closed = [closed zeros(size_closed,2)];

%grid = ones(ceil(range_y/stepsize),ceil(range_x/stepsize));
%sub2ind(size(grid), range_y+offsety-closed(:,2), -offsetx+1+closed(:,1));
%grid(sub2ind(size(grid), ceil((range_y+offsety-closed(:,2))/stepsize), ceil((-offsetx+1+closed(:,1))/stepsize) )) = -1;
%to do check if inital poistion is valid...
%put start on open
open_count = 1;
 node_no = 1;
 node_par = 1;
open(open_count,:) = [ node_no node_par floor(abs( (temp_current(1) + 1i*temp_current(2)) - (target(1) +1i*target(2)))) 0 floor(abs( (temp_current(1) + 1i*temp_current(2)) - (target(1) +1i*target(2)))) ];
%shortest path method
condition =1;

gvalue = open(1,4);
test =0;
 open(1,:) =[];
 nosol = 0;

while(condition)
   
    size_closed = size_closed +1;
    open_count = size(open,1) +1;
    if((~isempty(node_no)))
    closed(size_closed,:) = [node_no node_par];
     adjacency_new(1:end-2,node_no) = Inf;
   
   % grid(range_y+offsety-ypos,-offsetx+1+xpos) =-1;
    test = test+1;
    end
    %to do check if inital poistion is valid...
     
   % child = succ( [xpos ypos gvalue], grid,offsetx,offsety, range_y,target,robot);
    child = succ_vis(  node_no, adjacency_new,gvalue, target,robot);
    
    if(~isempty(child))
    open(open_count: open_count + size(child,1)-1,:)= child;
    end
   node_no=[];
  
    if(~isempty(open))
    open = sortrows(open,5);
    node_no = open(1,1);
    
    node_par = open(1,2);
    
    gvalue = open(1,4);
    
    
    
    %remove open location wih same position
    index_delete = find((open(:,1)==node_no));
    open(index_delete,:)=[];
    if(node_no== 2)
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
    optimal(1,:) =[adjacency_new(end-1,node_no) adjacency_new(end,node_no)];
    while(~((node_par== 1)))
        inc = inc +1;
        optimal(inc,:) =[adjacency_new(end-1,node_par) adjacency_new(end,node_par)];
        
        row_par = find((closed(:,1)==node_par));
        node_par = closed(row_par ,2);
        
    end
    
    
end
%check if target and start are what is written in adjcency map
optimal = [adjacency_new(end-1,1) adjacency_new(end,1); optimal(end:-1:1,:)];
if( roundn(currentPos(1),-5) ~= roundn(adjacency_new(end-1,1),-5))||( roundn(currentPos(2),-5) ~= roundn(adjacency_new(end,1),-5))
optimal = [currentPos(1)  currentPos(2) ;optimal];
start = 1;
end
if( roundn(target_real(1),-5) ~= roundn(adjacency_new(end-1,2),-5))||( roundn(target_real(2),-5) ~= roundn(adjacency_new(end,2),-5))
optimal = [optimal; target_real(1)  target_real(2)];
final=1;
end

x_diff = diff(optimal(:,1));
y_diff = diff(optimal(:,2));



move_commands = abs(real(x_diff) + 1j.*real(y_diff));
turn_commands = atan2(real(y_diff),real(x_diff));

%move in steps of 5
if (stepsize == 0)
    output_move = move_commands;
    output_angle = turn_commands;
else
output_move = [];
output_angle = [];
for iteration = 1 + start: size(move_commands,1)-final
    last = mod(move_commands(iteration), stepsize);
    steps_needed = round((move_commands(iteration)-last)/stepsize);
    
    if(roundn(last,-5) == 0)
        
        last =[];
    end
    output_move = [output_move ;repmat(stepsize,steps_needed,1); last];
    output_angle = [ output_angle; repmat(turn_commands(iteration),steps_needed + size(last,1),1)];
    
end
if(start == 1)
    output_move= [move_commands(1);output_move];
    output_angle = [turn_commands(1); output_angle];
end
if(final ==1)
    output_move= [output_move; move_commands(end)];
    output_angle = [output_angle; turn_commands(end)];
end
end
end


