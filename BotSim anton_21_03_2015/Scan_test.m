%An example of the ultrasound scanning function
clf;        %clears figures
clc;        %clears console
clear;      %clears workspace

COM_CloseNXT all;  
h = COM_OpenNXT();
COM_SetDefaultNXT(h);

OpenUltrasonic(SENSOR_4); %open usensor on port 4
mot = NXTMotor('B');  %motor connected to port C
mot.ResetPosition();  %only do this once at the start
mot.SmoothStart = 1;

%performs a 360 degree scan at 20% power with plotting on
%[rad rad2 ang] = ultraScanCCW(40,16)
%[rad rad1 rad2 ang] = ultraScan(30,60)
[rad ang] = ultraScan(30,60)
% plots the results
polar(ang/(360)*2*pi,rad1','-rx');
%hold on
%polar(ang/(360)*2*pi,rad2','-bx');
%polar(ang/(360)*2*pi,rad','-gx');
