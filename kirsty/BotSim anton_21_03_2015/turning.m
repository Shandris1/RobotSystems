function [] = turning(speed,angle)
%positive angle turns right
%motor.stop('Brake'); %cancels any previous movement that may be happening
mA = NXTMotor('A')
mC = NXTMotor('C')
mA.ActionAtTachoLimit = 'Brake'; % Want precise movment
mC.ActionAtTachoLimit = 'Brake';


angle = mod(angle+ 2*pi,2*pi)

if  angle > pi
     %so it can be send negative values as well
     angle = 2*pi -angle
    speed = speed * -1;
end
% 
% if angle < 0 || angle > pi
%     angle = abs (angle); %so it can be send negative values as well
%     speed = speed * -1;
% end
% if angle > pi
%     angle = angle - pi
% end


mA.Power = -speed; % so it turns counterclockwise
mC.Power = speed;
mA.SmoothStart = true; %we want to avoid slipping
mC.SmoothStart = true;
DriveD = round(((angle/pi*180) * 4.2 ))
if(DriveD~=0)
mA.TachoLimit = DriveD;
mC.TachoLimit = DriveD;
mA.SendToNXT(); %move motor
mC.SendToNXT();
mC.WaitFor();
mA.WaitFor();
end
end