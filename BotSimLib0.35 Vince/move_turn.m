function [botSim,particles,par_pos,par_ang] = move_turn(botSim,particles,num,moveCmd,turnCmd,mean_ang,par_pos,par_ang)

delta = turnCmd - mean_ang
botSim.turn(delta);
%botSim.move(moveCmd);
%currentAng = turn;
%currentPos = botSim.getBotPos();
%x =[currentPos(1) currentPos(1)+cos(currentAng)*3];
%y = [currentPos(2) currentPos(2)+sin(currentAng)*3];
botSim.drawBot(3,'g');
%refreshdata(h1,'caller')
%refreshdata(h2,'caller')

for i=1:num
    particles(i).turn(delta);
    %particles(i).move(moveCmd); 
    
    par_pos(i,:) =  particles(i).getBotPos;
    par_ang(i) = particles(i).getBotAng;

    par_ang(i) =  mod((par_ang(i)+2*pi),2*pi);
end

drawnow

end