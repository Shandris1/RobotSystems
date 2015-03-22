function [] = driving(speed,distance)
% Setting up initial parameters
mL   = MOTOR_A;
mR  = MOTOR_C;
bothMotors  = [mL; mR];
if (distance < 0)
   distance = distance*-1;
   speed = speed*-1; 
end
%turning distance in cm into motor degrees
drivingDist  = round(distance * 27.7) ; % in motor degrees
if(drivingDist~=0)

% creating a driving object
mForward = NXTMotor(bothMotors, 'Power', speed, 'TachoLimit', drivingDist);
% sending the robot forward
mForward.SendToNXT();
%waiting for results
mForward.WaitFor();
% done!
end
end