
SimulationTime      = 5;
times               = [0:Ts:SimulationTime];

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
                
RtfUntruncated          = idfrd( real(Rtransfer), imag(Rtransfer), SimulationTime );
                   
Rtf                     = tfest(RtfUntruncated,2,1); 
Rtf3rdOrder             = tfest(RtfUntruncated,3,2); 
Rtf4thOrder             = tfest(RtfUntruncated,4,3); 

outR                    = OutputLog{1}.Values.Data;

%plot(Rtf)
semilogx(Rfrequencies, 10*log10(Rtransfer))
title("Transfer function in the R axis");
xlabel("Frequency [Hz]");
ylabel("Gain [dB]");

plot(times,outR(:,1))
title("Response to step function R axis (d = 0.01 s)");
xlabel("Time [s]");
ylabel("R angle [radians]");
%bode(Rtf, fs * 2 * pi);
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
                   
XtfUntruncated          = idfrd( real(Xtransfer), imag(Xtransfer), SimulationTime );
                   
Xtf                     = tfest(XtfUntruncated,2,1);        
Xtf3rdOrder             = tfest(XtfUntruncated,3,2); 
Xtf4thOrder             = tfest(XtfUntruncated,4,3);       

outX                    = OutputLog{1}.Values.Data;

%plot(Xtf)
semilogx(Xfrequencies, 10*log10(Xtransfer))
title("Transfer function in the X axis");
xlabel("Frequency [Hz]");
ylabel("Gain [dB]");

plot(times,outX(:,2))
title("Response to step function X axis (d = 0.01 s)");
xlabel("Time [s]");
ylabel("X angle [radians]");
%bode(Xtf, fs * 2 * pi);
% 
% plot(Xtransfer)
% title("X axis response transfer function");
% hold off
% 
% plot(InputLog{1}.Values.Data(:,2))
% hold on
% plot(OutputLog{1}.Values.Data(:,2))
% legend("Input","Output")
% title("X step function response")
% hold off
% 
% 
% 
% %bode(Rtf)
% %nyquist(Rtf)
% 
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
                   
ZtfUntruncated          = idfrd( real(Ztransfer), imag(Ztransfer), SimulationTime );
                   
Ztf                     = tfest(ZtfUntruncated,2,1);        
Ztf3rdOrder             = tfest(ZtfUntruncated,3,2); 
Ztf4thOrder             = tfest(ZtfUntruncated,4,3); 

outZ                    = OutputLog{1}.Values.Data;

%plot(Ztf)
semilogx(Zfrequencies, 10*log10(Ztransfer))
title("Transfer function in the Z axis");
xlabel("Frequency [Hz]");
ylabel("Gain [dB]");

plot(times,outZ(:,3))
title("Response to step function Z axis (d = 0.01 s)");
xlabel("Time [s]");
ylabel("Z angle [radians]");
%bode(Ztf, fs * 2 * pi);
% 
% plot(Ztransfer)
% title("Z axis response transfer function");
% hold off
% 
% plot(InputLog{1}.Values.Data(:,3))
% hold on
% plot(OutputLog{1}.Values.Data(:,3))
% legend("Input","Output")
% title("Z step function response")
% hold off
% 
% 
% 
% %bode(Rtf)
% %nyquist(Rtf)