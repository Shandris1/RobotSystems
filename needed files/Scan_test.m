%An example of the ultrasound scanning function
clf;        %clears figures
clc;        %clears console
clear;      %clears workspace

COM_CloseNXT all;  
h = COM_OpenNXT();
COM_SetDefaultNXT(h);

OpenUltrasonic(SENSOR_4); %open usensor on port 4
mot = NXTMotor('B');  %motor connected to port C
%mot.ResetPosition();  %only do this once at the start
mot.SmoothStart = 0;

%performs a 360 degree scan at 20% power with plotting on

[rad rad2 ang] = ultraScanCW(20,16)


% plots the results
polar(ang/(360)*2*pi,rad','-rx');
hold on
polar(ang/(360)*2*pi,rad2','-bx');