
SimulationTime      = 30;
times               = [0:Ts:SimulationTime];
OneHundredlimit     = 3003;

%R axis
waypoints           = [ 0 0 1 1; 0.3 0.3 0.3 0.3; 0 0 0 0];
times               = [0 0.99 1 5];

waypointTimes       = times;                                    
endtime             = times(length(times));                         %time at which all movement (nominally) stops
trajTimes           = [0 : 0.00048828125 : endtime];                %time for each calculated step in the movement (very specific number)
waypointAccelTimes  = ones(1,length(waypointTimes) - 1);            %acceleration settings for each way point (always 1)
[q]                 = trapveltraj(waypoints,numel(trajTimes));      %generalised coordinates

maxWaypoints        = length(times);
maxSize             = [3,maxWaypoints];

ReferenceR          = 0;
ReferenceX          = 0.3;
ReferenceZ          = 0;

open_system('RobotArm')
model_workspace = get_param('RobotArm','ModelWorkspace');
sim('RobotArm.slx')

%estimation
[Rtransfer, Rfrequencies] = tfestimate(InputLog{1}.Values.Data(:,1), ...
                       OutputLog{1}.Values.Data(:,1), ...
                       hann(length(InputLog{1}.Values.Data(:,1))), ...
                       0, ...
                       length(InputLog{1}.Values.Data(:,1)), ...
                       fs);
                
RtfUntruncated          = idfrd( real(Rtransfer(1:OneHundredlimit)), imag(Rtransfer(1:OneHundredlimit)), SimulationTime );

Rtf2zeroes              = tfest(RtfUntruncated,4,2);
Rtf3zeroes              = tfest(RtfUntruncated,4,3);

outR                    = OutputLog{1}.Values.Data;

%plot(Rtf)
semilogx(Rfrequencies, 10*log10(Rtransfer))
title("Transfer function in the R axis");
xlabel("Frequency [Hz]");
ylabel("Gain [dB]");

% plot(times,outR(:,1))
% title("Response to step function R axis (d = 0.01 s)");
% xlabel("Time [s]");
% ylabel("R angle [radians]");
% bode(Rtf, fs * 2 * pi);
% title("R axis response transfer function");
% hold off
% 
% plot(InputLog{1}.Values.Data(:,1))
% hold on
% plot(OutputLog{1}.Values.Data(:,1))
% legend("Input","Output")
% title("R step function response")
% hold off
% 
% 
% 
% %bode(Rtf)
% %nyquist(Rtf)
% 
%%
%X axis
waypoints           = [ 0 0 0 0; 0.3 0.3 0.5 0.5; 0 0 0 0];
times               = [0 0.99 1 5];

waypointTimes       = times;                                    
endtime             = times(length(times));                         %time at which all movement (nominally) stops
trajTimes           = [0 : 0.00048828125 : endtime];                %time for each calculated step in the movement (very specific number)
waypointAccelTimes  = ones(1,length(waypointTimes) - 1);            %acceleration settings for each way point (always 1)
[q]                 = trapveltraj(waypoints,numel(trajTimes));      %generalised coordinates

maxWaypoints        = length(times);
maxSize             = [3,maxWaypoints];

ReferenceR          = 0;
ReferenceX          = 0.3;
ReferenceZ          = 0;

open_system('RobotArm')
model_workspace = get_param('RobotArm','ModelWorkspace');
sim('RobotArm.slx')

%estimation
[Xtransfer, Xfrequencies] = tfestimate(InputLog{1}.Values.Data(:,2), ...
                       OutputLog{1}.Values.Data(:,2), ...
                       hann(length(InputLog{1}.Values.Data(:,2))), ...
                       0, ...
                       length(InputLog{1}.Values.Data(:,2)), ...
                       fs);
                   
XtfUntruncated          = idfrd( real(Xtransfer(1:OneHundredlimit)), imag(Xtransfer(1:OneHundredlimit)), SimulationTime );

Xtf2zeroes              = tfest(XtfUntruncated,4,2);
Xtf3zeroes              = tfest(XtfUntruncated,4,3);

outX                    = OutputLog{1}.Values.Data;

%plot(Xtf)
semilogx(Xfrequencies, 10*log10(Xtransfer))
title("Transfer function in the X axis");
xlabel("Frequency [Hz]");
ylabel("Gain [dB]");

%%
%Z axis
waypoints           = [ 0 0 0 0; 0.3 0.3 0.3 0.3; -0.35 -0.35 0.25 0.25];
times               = [0 0.99 1 5];

waypointTimes       = times;                                    
endtime             = times(length(times));                         %time at which all movement (nominally) stops
trajTimes           = [0 : 0.00048828125 : endtime];                %time for each calculated step in the movement (very specific number)
waypointAccelTimes  = ones(1,length(waypointTimes) - 1);            %acceleration settings for each way point (always 1)
[q]                 = trapveltraj(waypoints,numel(trajTimes));      %generalised coordinates

maxWaypoints        = length(times);
maxSize             = [3,maxWaypoints];

ReferenceR          = 0;
ReferenceX          = 0.3;
ReferenceZ          = 0;

open_system('RobotArm')
model_workspace = get_param('RobotArm','ModelWorkspace');
sim('RobotArm.slx')

%estimation
[Ztransfer, Zfrequencies] = tfestimate(InputLog{1}.Values.Data(:,3), ...
                       OutputLog{1}.Values.Data(:,3), ...
                       hann(length(InputLog{1}.Values.Data(:,3))), ...
                       0, ...
                       length(InputLog{1}.Values.Data(:,3)), ...
                       fs);

ZtfUntruncated          = idfrd( real(Ztransfer(1:OneHundredlimit)), imag(Ztransfer(1:OneHundredlimit)), SimulationTime );

Ztf2zeroes              = tfest(ZtfUntruncated,4,2);
Ztf3zeroes              = tfest(ZtfUntruncated,4,3);

outZ                    = OutputLog{1}.Values.Data;

%plot(Ztf)
semilogx(Zfrequencies, 10*log10(Ztransfer))
title("Transfer function in the Z axis");
xlabel("Frequency [Hz]");
ylabel("Gain [dB]");