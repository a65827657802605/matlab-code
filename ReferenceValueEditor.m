% MAKE SURE YOU SET THE VALUES FOR THE REFERENCE INPUTS (R, Z, X) TO THE
% VARIABLES 'ReferenceR', 'ReferenceZ' AND 'ReferenceX' IN THE SIMULINK
% FILE.

%%
%Input
%See the image for a definition of the coordinate systems

x                       = [0.25];    %[m] x postition as defined in figure
y                       = [0.25];  %[m] y postition as defined in figure
h                       = [0.25];    %[m] desired height
times                   = [1];               %[s] time at which the gripper needs to arrive at the position 

robot_to_conveyorbelt   = 0.05;                     %[m] distance from the robot to the conveyorbelt

%See Rutger's drawing
gripper_length          = 0.02;                        %[m] distance between the gripper attachment point and the robot arm
AB                      = 0.081;                    %[m]
BC                      = 0.165;                    %[m]
BG                      = 0.080;                    %[m]
CD                      = 0.130;                    %[m]
DE                      = 0.120;                    %[m]
EF                      = 0.065;                    %[m]
GE                      = 0.130;                    %[m]

%%
%Transforming every coordinate
coordinates             = [x; y; h];
[three,n_coordinates]   = size(coordinates);
waypoints               = zeros(3, n_coordinates);             %[R, X, Z]

% for i = 1:n_coordinates
% 
%     coordinate          = coordinates(:,i)';
%     intermediate_coordinate = [0 0 0];                   %[R, y, r]
%     waypoint            = [0 0 0];                       %[R, X, Z]
%      %Transpose the matrix
%     
%     %%
%     %Conversion from (x,y) to (R,r)
%     
%     x                           = coordinate(1);
%     y                           = coordinate(2);
%     
%     intermediate_coordinate(2)  = y + robot_to_conveyorbelt; %Shifting origin
%     y                           = intermediate_coordinate(2);
%     intermediate_coordinate(3)  = sqrt( x * x + y * y ); %Pythagorean theorem
%     intermediate_coordinate(1)  = atan2( y, x ); %Standard arctangent rule
%     
%     r                           = intermediate_coordinate(3);
%     R                           = intermediate_coordinate(1);
%     h                           = coordinate(3);
%     %%
%     %Check if constraints are met
% 
%     boundsArccos = ( -315 < r < -55 && 246 - sqrt(99225 - r^2) < h < 246 + sqrt(99225 - r^2) ) || ...
%                    ( -55 <= r <= 55 && (246 - sqrt(99225 - r^2) < h < 246 - sqrt(3025 - r^2) || 246 + sqrt(3025 - r^2) < h < 246 + sqrt(99225 - r^2)) ) || ...
%                    ( 55 < r < 315 && 246 - sqrt(99225 - r^2) < h < 246 + sqrt(99225 - r^2));
% 
%     if (~boundsArccos)
%         error("According to the arccos constraint, one of the inputs is out of bounds.")
%     end
% 
%     %%
%     %Conversion from (r,h) to (X,Z)
% 
%     AC                      = BC + AB;
%     DF                      = DE + EF;
% 
%     CF                      = sqrt( r * r + (AC - h - gripper_length) ^ 2 ); 
%     phi1                    = atan2( r, (AC - h + gripper_length));
%     phi2                    = acos((CF^2 + DF^2 - CD^2) / (2 * CF * DF));
%     phi3                    = pi / 2 - phi1 - phi2;
%     Dx                      = r - DF * sin(phi3);
%     Dy                      = h + DF * cos(phi3);
% 
%     Dy1                     = Dy - (AB - AC);
%     phi4                    = atan2( Dy - h - gripper_length, r - Dx);
%     Ex                      = r - EF * sin(phi4);
%     Ey                      = EF * cos(phi4) + h;
% 
%     Ey1                     = Ey - AB;
%     BE                      = sqrt( Ex^2 + Ey1^2 );
% 
%     Z                       = asin( Dy1 / CD );
%     X                       = atan2( Ex, Ey1 ) - acos( (BG^2 + BE^2 - GE^2) / 2 / BG / BE);
% 
%     %%
%     %Check if constraints are met
% 
%     boundsR                 = (R > (-1 * pi / 2) && R < (pi / 2));
%     boundsZ                 = (Z > -0.4 && Z < 0.3);
%     boundsX                 = (X > 0.2 && X < 1.47);
% 
%     if (boundsR)
%         waypoint(1) = R;
%     else
%         error("Reference angle R is out of bounds")
%     end
% 
%     if (boundsX)
%         waypoint(2) = X;
%     else
%         error("Reference angle X is out of bounds")
%     end
% 
%     if (boundsZ)
%         waypoint(3) = Z;
%     else
%         error("Reference angle Z is out of bounds")
%     end
% 
%     waypoints(:,i)      = waypoint'; %[R, X, Z]
%     
% end

%%
%Updating the model parameters

waypoints           = [0 0 0 0; 0.025113778764355 0.025113778764355 0.025113778764355 0.025113778764355; 0.5 0.5 0.5 0.5];
times               = [0 6 12 18];

waypointTimes       = times;                                    
endtime             = times(length(times));                         %time at which all movement (nominally) stops
trajTimes           = [0 : 0.00048828125 : endtime];                %time for each calculated step in the movement (very specific number)
waypointAccelTimes  = ones(1,length(waypointTimes) - 1);            %acceleration settings for each way point (always 1)
[q]                 = trapveltraj(waypoints,numel(trajTimes));      %generalised coordinates

maxWaypoints        = length(times);
maxSize             = [3,maxWaypoints];


%[q,qd,qdd] = trapveltraj(waypoints,numel(trajTimes), ...
%    'AccelTime',repmat(waypointAccelTimes,[3 1]), ... 
%    'EndTime',repmat(diff(waypointTimes),[3 1]));

ReferenceR          = 0;
ReferenceX          = 0.3;
ReferenceZ          = 0;

%%
%Run the model: (NEEDS TO BE AT THE END OF THE SCRIPT)
open_system('RobotArm')
model_workspace = get_param('RobotArm','ModelWorkspace');
sim('RobotArm.slx')