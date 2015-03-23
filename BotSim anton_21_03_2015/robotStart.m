function [] = robotStart()
COM_CloseNXT all;  
h = COM_OpenNXT();
COM_SetDefaultNXT(h);
OpenUltrasonic(SENSOR_4);

port  = MOTOR_B;

%assign sensorMotor to NXT Motor B 
sensorMotor   = NXTMotor(port, 'Power', -100, 'ActionAtTachoLimit', 'Brake');
sensorMotor.Stop('off');
sensorMotor.ResetPosition();

end