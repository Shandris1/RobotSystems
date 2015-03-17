function [botSim2, botSim] = circ_shift(botSim2, botSim)
draw = 1;
    if(draw)   

             if botSim.debug()
                hold off; %the drawMap() function will clear the drawing when hold is off
                botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
                botSim.drawBot(30,'g'); %draw robot with line length 30 and green

                    if(botSim2.insideMap == 1)
                        botSim2.drawBot(3); %draw particle with line length 3 and default color
                    end
        
                drawnow;
             end
    end
    

    readings = 6;
    parScan = botSim2.ultraScan();
    botScan = botSim.ultraScan();

    for j =0 :readings-1
        %check_orient(j+1)  = max((abs(circshift(parScan,j) - botScan))./botScan);
        %check_orient2(j+1) = sqrt(sum((botScan - circshift(parScan,j)) .^ 2)); 
        check_orient(j+1) = sum((abs(circshift(parScan,j) - botScan))./botScan);
    end
    [lowest, best] = min((check_orient));
    %dist(i) = sqrt(sum((botScan - circshift(parScan,best - 1)) .^ 2)); 
    if(lowest < 0.1)
        botSim2.turn(-2*pi/readings * (best-1));      
        parScan = circshift(parScan,best-1);
    end
    
    
    if(draw)   

         if botSim.debug()
            hold off; %the drawMap() function will clear the drawing when hold is off
            botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
            botSim.drawBot(30,'g'); %draw robot with line length 30 and green
       
                if(botSim2.insideMap == 1)
                    botSim2.drawBot(3); %draw particle with line length 3 and default color
                end
      
            drawnow;
         end
    end

end