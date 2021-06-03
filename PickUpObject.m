function PickUpObject(cameraX, cameraY, detectionTime, conveyorSpeed)
    
    %Load predetermined values into memory
    offsetCameraOriginX     = -0.172;                   %[m]
    offsetCameraOriginY     = 0.069;                    %[m]
    robot_to_conveyorbelt   = 0.05;                     %[m] distance from the robot to the conveyorbelt
    height_conveyorbelt     = 0.054;                    %[m] NEEDS TO BE CHECKED
    height_bag              = 0.035;                    %[m] NEEDS TO BE CHECKED
    airborne_height         = 0.05;                     %[m] how high we want to lift it from the ground
    bag_resting_distance    = 0.20;                     %[m] how far away from the centre of the robot the bag will be put
    
    %1- above conveyorbelt -2- grabbing bag -3- sucking up bag -4- picking 
    %up bag -5- swinging anti-clockwise -6- putting the bag down -7- back 
    %up -8- swinging clockwise -9.
    %Time intervals [s]:
    T12                     = 0.7;
    T23                     = 0.3;
    T34                     = 0.2;
    T45                     = T23;
    T56                     = 0.6;
    T67                     = 0.3;
    T78                     = T67;
    T89                     = T56;
    
    %See Rutger's drawing
    gripper_length          = 0.02;                     %[m] distance between the gripper attachment point and the robot arm
    AB                      = 0.081;                    %[m]
    BC                      = 0.165;                    %[m]
    BG                      = 0.080;                    %[m]
    CD                      = 0.130;                    %[m]
    DE                      = 0.120;                    %[m]
    EF                      = 0.065;                    %[m]
    GE                      = 0.130;                    %[m]
    
    %Change the origin of the coordinates to match the origin
    [currentX, currentY]    = ChangeCameraCoordinateOrigin(cameraX, cameraY);
    
    %Compute the arrival time of the package when it is right in front of
    %the robot arm
    arrivalTime             = ArrivalTime(currentX, detectionTime, conveyorSpeed);
    
    %Go from the (x, y) coordinate system to a (R, r) coordinate system
    [R, r]                  = FromXYtoRR(currentY);

    %Go from (R, r, h) coordinates to (X, Z, R)
    [X, Z]                  = FromRHtoXZ(r, height_conveyorbelt + height_bag + 0.02);
    [Xcontact, Zcontact]    = FromRHtoXZ(r, height_conveyorbelt + height_bag);
    [Xairborne, Zairborne]  = FromRHtoXZ(r, height_conveyorbelt + height_bag + airborne_height);
    [Xdown, Zdown]          = FromRHtoXZ(r, height_bag);
    
    %Obtain the [R, X, Z] coordinates for each point in the movement
    %trajectory
    waypoints               = DetermineWaypoints(R, X, Z, Xcontact, Zcontact, Xairborne, Zairborne, Xdown, Zdown);
    
    %Obtain the table of times it needs to arrive at certain places
    times                   = DetermineTimes(arrivalTime, T12, T23, T34, T45, T56, T67, T78, T89);

    %Update the model parameters.
    UpdateParameters(times, waypoints);

end

function [currentX, currentY] = ChangeCameraCoordinateOrigin(cameraX, cameraY)

    currentX = cameraX + offsetCameraOriginX;
    currentY = cameraY + offsetCameraOriginY;

end

function arrivalTime = ArrivalTime(currentX, detectionTime, conveyorSpeed)

    if (conveyorSpeed == -1)
          velocity = 0.04;
    else
          velocity = 0.077;
    end

    % t = Delta x / v + t0; Delta x = 0 - x = -x
    arrivalTime = -1 * currentX / velocity + detectionTime;

end

function [R, r] = FromXYtoRR(currentY)
    
    %When the object needs to be picked up, it will always be in the middle
    R = 0;
    %The distance from the robot
    r = currentY + robot_to_conveyorbelt;
    
end

function [X, Z] = FromRHtoXZ(r, h)

    CheckArccosConstraint(r, h);

    AC                      = BC + AB;
    DF                      = DE + EF;

    CF                      = sqrt( r * r + (AC - h - gripper_length) ^ 2 ); 
    phi1                    = atan2( r, (AC - h + gripper_length));
    phi2                    = acos((CF^2 + DF^2 - CD^2) / (2 * CF * DF));
    phi3                    = pi / 2 - phi1 - phi2;
    Dx                      = r - DF * sin(phi3);
    Dy                      = h + DF * cos(phi3);

    Dy1                     = Dy - (AB - AC);
    phi4                    = atan2( Dy - h - gripper_length, r - Dx);
    Ex                      = r - EF * sin(phi4);
    Ey                      = EF * cos(phi4) + h;

    Ey1                     = Ey - AB;
    BE                      = sqrt( Ex^2 + Ey1^2 );

    Z                       = asin( Dy1 / CD );
    X                       = atan2( Ex, Ey1 ) - acos( (BG^2 + BE^2 - GE^2) / 2 / BG / BE);
    
    CheckXZConstraint(X, Z);

end

function CheckArccosConstraint(r, h)

    boundsArccos = ( -315 < r < -55 && 246 - sqrt(99225 - r^2) < h < 246 + sqrt(99225 - r^2) ) || ...
                   ( -55 <= r <= 55 && (246 - sqrt(99225 - r^2) < h < 246 - sqrt(3025 - r^2) || 246 + sqrt(3025 - r^2) < h < 246 + sqrt(99225 - r^2)) ) || ...
                   ( 55 < r < 315 && 246 - sqrt(99225 - r^2) < h < 246 + sqrt(99225 - r^2));

    if (~boundsArccos)
        error('According to the arccos constraint, one of the inputs is out of bounds.')
    end

end

function CheckXZConstraint(X, Z)

    boundsZ                 = (Z > -0.4 && Z < 0.3);
    boundsX                 = (X > 0.2 && X < 1.47);

    if (boundsX)
    else
        error('Reference angle X is out of bounds')
    end

    if (boundsZ)
    else
        error('Reference angle Z is out of bounds')
    end

end

function waypoints = DetermineWaypoints(R, X, Z, Xcontact, Zcontact, Xairborne, Zairborne, Xdown, Zdown)

        %1- above conveyorbelt -2- grabbing bag -3- sucking up bag -4- picking 
    	%up bag -5- swinging anti-clockwise -6- putting the bag down -7- back 
        %up -8- swinging clockwise -9.

        %[R; X; Z]

        waypoints = [ R, R,        R,        R,         R - pi / 2, R - pi / 2, R - pi / 2, R         ;...
                      X, Xcontact, Xcontact, Xairborne, Xairborne,  Xdown,      Xairborne,  Xairborne ;...
                      Z, Zcontact, Zcontact, Zairborne, Zairborne,  Zdown,      Zairborne,  Zairborne ];

end

function times = DetermineTimes(arrivalTime, T12, T23, T34, T45, T56, T67, T78, T89)

    %1- above conveyorbelt -2- grabbing bag -3- sucking up bag -4- picking 
    %up bag -5- swinging anti-clockwise -6- putting the bag down -7- back 
    %up -8- swinging clockwise -9.
    
    times = [   arrivalTime - T12 - T23                                 ,...
                arrivalTime - T23                                       ,...
                arrivalTime                                             ,...
                arrivalTime + T34                                       ,...
                arrivalTime + T34 + T45                                 ,...
                arrivalTime + T34 + T45 + T56                           ,...
                arrivalTime + T34 + T45 + T56 + T67                     ,...
                arrivalTime + T34 + T45 + T56 + T67 + T78               ,...
                arrivalTime + T34 + T45 + T56 + T67 + T78 + T89         ];
                
end

function UpdateParameters(times, waypoints)

    waypointTimes       = times;                                    
    endtime             = times(length(times));                         %time at which all movement (nominally) stops
    trajTimes           = [0 : 0.00048828125 : endtime];                %time for each calculated step in the movement (very specific number)
    waypointAccelTimes  = ones(1,length(waypointTimes) - 1);            %acceleration settings for each way point (always 1)
	[q]                 = trapveltraj(waypoints,numel(trajTimes));      %generalised coordinates
    maxWaypoints        = length(times);
    maxSize             = [3,maxWaypoints];

end