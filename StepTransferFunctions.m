% %Please run the other file first
% 
% %R axis
% waypoints           = [ 0 0 1 1; 0.3 0.3 0.3 0.3; 0 0 0 0];
% times               = [0 0.99 1 5];
% 
% waypointTimes       = times;                                    
% endtime             = times(length(times));                         %time at which all movement (nominally) stops
% trajTimes           = [0 : 0.00048828125 : endtime];                %time for each calculated step in the movement (very specific number)
% waypointAccelTimes  = ones(1,length(waypointTimes) - 1);            %acceleration settings for each way point (always 1)
% [q]                 = trapveltraj(waypoints,numel(trajTimes));      %generalised coordinates
% 
% maxWaypoints        = length(times);
% maxSize             = [3,maxWaypoints];
% 
% ReferenceR          = 0;
% ReferenceX          = 0.3;
% ReferenceZ          = 0;
% 
% open_system('RobotArm')
% model_workspace = get_param('RobotArm','ModelWorkspace');
% sim('RobotArm.slx')
% 
% %estimation
% Rtransfer = tfestimate(InputLog{1}.Values.Data(:,1), ...
%                        OutputLog{1}.Values.Data(:,1), ...
%                        hann(length(InputLog{1}.Values.Data(:,1))), ...
%                        0, ...
%                        length(InputLog{1}.Values.Data(:,1)), ...
%                        fs);
% Rtf = minreal(tf(Rtransfer));
%                   
% plot(Rtransfer)
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
% %%
%X axis
% waypoints           = [ 0 0 0 0; 0.3 0.3 0.5 0.5; 0 0 0 0];
% times               = [0 0.99 1 5];
% 
% waypointTimes       = times;                                    
% endtime             = times(length(times));                         %time at which all movement (nominally) stops
% trajTimes           = [0 : 0.00048828125 : endtime];                %time for each calculated step in the movement (very specific number)
% waypointAccelTimes  = ones(1,length(waypointTimes) - 1);            %acceleration settings for each way point (always 1)
% [q]                 = trapveltraj(waypoints,numel(trajTimes));      %generalised coordinates
% 
% maxWaypoints        = length(times);
% maxSize             = [3,maxWaypoints];
% 
% ReferenceR          = 0;
% ReferenceX          = 0.3;
% ReferenceZ          = 0;
% 
% open_system('RobotArm')
% model_workspace = get_param('RobotArm','ModelWorkspace');
% sim('RobotArm.slx')
% 
% %estimation
% Xtransfer = tfestimate(InputLog{1}.Values.Data(:,2), ...
%                        OutputLog{1}.Values.Data(:,2), ...
%                        hann(length(InputLog{1}.Values.Data(:,2))), ...
%                        0, ...
%                        length(InputLog{1}.Values.Data(:,2)), ...
%                        fs);
% Xtf = minreal(tf(Xtransfer));
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
% %%
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
% 
% %estimation
% Ztransfer = tfestimate(InputLog{1}.Values.Data(:,3), ...
%                        OutputLog{1}.Values.Data(:,3), ...
%                        hann(length(InputLog{1}.Values.Data(:,3))), ...
%                        0, ...
%                        length(InputLog{1}.Values.Data(:,3)), ...
%                        fs);
%                    
% Ztf = minreal(tf(Ztransfer));
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