function [radii, angles] = drivingtest(speed,distance)
% Set some parameters:
leftWheel   = MOTOR_A;
rightWheel  = MOTOR_C;
bothWheels  = [leftWheel; rightWheel];
drivingPower = speed;
turningPower = 40;
drivingDist  = round(distance * 27.7) ; % in degrees
turningDist  = 220;  % in degrees

% now create the objects for straigt driving:
mForward = NXTMotor(bothWheels, 'Power', drivingPower, 'TachoLimit', drivingDist);
mReverse = mForward; % clone object
mReverse.Power = -mForward.Power; % just swap the power sign

% let the bot drive forward
mForward.SendToNXT();
mForward.WaitFor();
% and now return to the origin

% done!
