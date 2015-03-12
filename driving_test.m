function [radii, angles] = drivingtest(speed,distance)
% Set some parameters:
leftWheel   = MOTOR_A;
rightWheel  = MOTOR_C;
bothWheels  = [leftWheel; rightWheel];
drivingPower = speed;
drivingDist  = round(distance * 27.7) ; % in degrees
% now create the objects for straigt driving:
mForward = NXTMotor(bothWheels, 'Power', drivingPower, 'TachoLimit', drivingDist);
% let the bot drive forward
mForward.SendToNXT();
mForward.WaitFor();
% and now return to the origin

% done!
