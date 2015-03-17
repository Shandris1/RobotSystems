function [moveCmd turnCmd optimal] = pathPlan(botSim,map,target,pos,ang)
    stepsize = 5;
    out = visbil_full(map,botSim,target);
    adjacency = visbil_point(map,target,out);
    [moveCmd turnCmd optimal] = Astar_visbil(pos,ang, adjacency,map,botSim,stepsize);
    
end