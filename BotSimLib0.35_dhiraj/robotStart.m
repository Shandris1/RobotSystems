function [] = robotStart()
COM_CloseNXT all;  
h = COM_OpenNXT();
COM_SetDefaultNXT(h);
OpenUltrasonic(SENSOR_4);

port  = MOTOR_B;

mLeft   = NXTMotor(port, 'Power', -20, 'ActionAtTachoLimit', 'Brake');
mRight  = NXTMotor(port, 'Power', 20, 'ActionAtTachoLimit', 'Brake');

mLeft.Stop('off');
mLeft.ResetPosition();
