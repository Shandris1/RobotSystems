function [radii, reading_point] = ultraScan(scanSpeed,samples)
%motor.stop('Brake'); %cancels any previous movement that may be happening


port  = MOTOR_B;
dist = round(360/samples);

mLeft   = NXTMotor(port, 'Power', scanSpeed, 'ActionAtTachoLimit', 'Brake');
mRight  = NXTMotor(port, 'Power', -scanSpeed, 'ActionAtTachoLimit', 'Brake');

mLeft.Stop('off');
radii = zeros(samples,1); %preallocate the matrix


    
    % where are we?
    data = mLeft.ReadFromNXT(); %read current encoder value
    pos  = -data.Position;
    mRight.TachoLimit = 360;
    mRight.SendToNXT();
    reading = 0;
    reading_point = -1;
    j=1;
    angles = 0;
    while (reading == 0)
        data = mRight.ReadFromNXT(); % doesn't matter which object we use to read!
        pos  = -data.Position;
        if( pos >= reading_point)
            radii(j) = GetUltrasonic(SENSOR_4);
            reading_point = reading_point + (360/samples);
            j = j+1;        
        elseif reading_point > 350
            reading = 1;
        end
    end
    
    mRight.WaitFor();
    data = mLeft.ReadFromNXT(); % doesn't matter which object we use to read!
    pos  = -data.Position;

    mLeft.TachoLimit = abs(pos);

    mLeft.SendToNXT();
    mLeft.WaitFor();
