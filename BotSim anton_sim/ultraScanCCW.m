function [radii, radii2, angles] = ultraScan(scanSpeed,samples)
%motor.stop('Brake'); %cancels any previous movement that may be happening


port  = MOTOR_B;
dist = round(360/samples);

mLeft   = NXTMotor(port, 'Power', scanSpeed, 'ActionAtTachoLimit', 'Brake');
mRight  = NXTMotor(port, 'Power', -scanSpeed, 'ActionAtTachoLimit', 'Brake');

mLeft.Stop('off');
radii = zeros(samples,1); %preallocate the matrix
angles = zeros(samples,1); %preallocate
radii2 = zeros(samples,1); %preallocate the matrix
test = 0;

    
    % where are we?
    data = mLeft.ReadFromNXT(); %read current encoder value
    pos  = -data.Position;
    mRight.TachoLimit = 360;
    mRight.SendToNXT();
    reading = 0;
    %reading_point = 0;
    j=1;
   % reading_point = reading_point + (360/samples);
    reading_point = 0 : (360/samples): 360- (360/samples);
            
     
    while (reading == 0)
        data = mRight.ReadFromNXT(); % doesn't matter which object we use to read!
        pos  = -data.Position;
        if( pos >= reading_point(j))
            radii(j) = GetUltrasonic(SENSOR_4);
             pos  = -data.Position;
        reading_point(j);
                    
       if(j == samples)
            reading = 1;
       end
       j = j+1;
        end
        test = test+1;
        
    end
    
    radii(radii>30) = radii(radii>30) +2;
   
                
          
    mRight.WaitFor();
    data = mLeft.ReadFromNXT(); % doesn't matter which object we use to read!
    pos  = -data.Position;
    
    %% copy pasted code
    
    % where are we?
    data = mLeft.ReadFromNXT(); % doesn't matter which object we use to read!
    pos  = -data.Position;
    mLeft.TachoLimit = 360;
    %radii2(1) = GetUltrasonic(SENSOR_4);
    %reading_point = reading_point - (360/samples);
    mLeft.SendToNXT();
    reading = 0;
    %reading_point = 360 - (360/samples);
     reading_point = 360 : -(360/samples): 0;
    j=1;
    while (reading == 0)
        data = mLeft.ReadFromNXT(); % doesn't matter which object we use to read!
        pos  = -data.Position;
        reading_point(j+1);
        if( pos <= reading_point(j+1))
            radii2(j) = GetUltrasonic(SENSOR_4);
             pos  = -data.Position;
        reading_point(j+1);
            %reading_point = reading_point - (360/samples);
           
            
                   
        if(j == samples)
            reading = 1;
        end
        j = j+1; 
       end
    end
     radii2(radii2>30) = radii2(radii2>30) +2;
     radii2 = (radii2(end:-1:1));
    mLeft.WaitFor();
    %mLeft.TachoLimit = abs(pos);
    radii;
    radii2;
    angles = 0 : (360/samples): 360- (360/samples);
     %mLeft.SendToNXT();
    % mLeft.WaitFor();
