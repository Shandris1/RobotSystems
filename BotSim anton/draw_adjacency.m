function draw_adjacency(adjacency,map)
figure
mapmod = [map; map(1,:)];
plot(mapmod(:,1),mapmod(:,2));
hold on
for i =1: size(adjacency,2)
    for j =1:size(adjacency,2)
        if(~isinf(adjacency(i,j)))
            plot([adjacency(end-1,i) adjacency(end-1,j)], [adjacency(end,i) adjacency(end,j)],'r');
        end
    end
end
end
