function [] = turning(speed,angle)
%positive angle turns right
%motor.stop('Brake'); %cancels any previous movement that may be happening
mA = NXTMotor('A')
mC = NXTMotor('C')
mA.ActionAtTachoLimit = 'Brake'; % Want precise movment
mC.ActionAtTachoLimit = 'Brake';
if angle < 0 
    angle = abs (angle); %so it can be send negative values as well
    speed = speed * -1;
end
mA.Power = -speed; % so it turns counterclockwise
mC.Power = speed;
mA.SmoothStart = true; %we want to avoid slipping
mC.SmoothStart = true;
DriveD = round(angle * 4.2) 
mA.TachoLimit = DriveD;
mC.TachoLimit = DriveD;
mA.SendToNXT(); %move motor
mC.SendToNXT();
mC.WaitFor();
mA.WaitFor();