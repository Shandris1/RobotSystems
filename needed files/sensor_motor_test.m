port  = MOTOR_B;


mLeft   = NXTMotor(port, 'Power', -100, 'ActionAtTachoLimit', 'Brake');
mRight  = NXTMotor(port, 'Power', 100, 'ActionAtTachoLimit', 'Brake');
    
i=0;
%mLeft.ResetPosition();
    mLeft.Stop('off');
    
    while (i<2)
    data = mLeft.ReadFromNXT(); %read current encoder value
    pos  = -data.Position
    mRight.TachoLimit = 360;
    mRight.SendToNXT();
        
    
    mRight.WaitFor();
    data = mLeft.ReadFromNXT(); %read current encoder value
    pos  = -data.Position
    mLeft.TachoLimit = 355;
    mLeft.SendToNXT();
    mLeft.WaitFor();
    i=i+1
    end
    data = mLeft.ReadFromNXT(); %read current encoder value
    pos  = -data.Position
    if (pos<0)
    mSlow   = NXTMotor(port, 'Power', -10, 'ActionAtTachoLimit', 'Brake');
    mSlow.TachoLimit = abs(pos);
    mSlow.SendToNXT();
    
    elseif (pos>0)
    mSlow   = NXTMotor(port, 'Power', 10, 'ActionAtTachoLimit', 'Brake');
    mSlow.TachoLimit = abs(pos);
    mSlow.SendToNXT();
    
    else
    end
    
    