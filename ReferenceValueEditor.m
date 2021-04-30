% MAKE SURE YOU SET THE VALUES FOR THE REFERENCE INPUTS (R, Z, X) TO THE
% VARIABLES 'ReferenceR', 'ReferenceZ' AND 'ReferenceX' IN THE SIMULINK
% FILE.

%%
%Input
%See the image for a definition of the coordinate systems

x                       = -0.3; %[m] x postition as defined in figure
y                       = 0.3; %[m] y postition as defined in figure
robot_to_conveyorbelt   = 0; %[m] distance from the robot to the conveyorbelt
h                       = 0.1; %[m] desired height

%See Rutger's drawing
gripper_height          = 0; %[m] distance between the gripper attachment point and the robot arm
AB                      = 0.081; %[m]
BC                      = 0.165; %[m]
BG                      = 0.080; %[m]
CD                      = 0.130; %[m]
DE                      = 0.120; %[m]
EF                      = 0.065; %[m]
GE                      = 0.130; %[m]

%%
%Conversion from (x,y) to (R,r)

y                       = y + robot_to_conveyorbelt; %Shifting origin
r                       = sqrt( x * x + y * y ); %Pythagorean theorem
R                       = atan( x / y ); %Standard arctangent rule

%%
%Conversion from (r,h) to (X,Z)

AC                      = BC + AB;
DF                      = DE + EF;

CF                      = sqrt( r * r + (AC - h + gripper_height) ^ 2 ); 
phi1                    = atan( (AC - h + gripper_height) / r );
phi2                    = acos( (CF^2 + DF^2 - CD^2) / (2 * CF * DF) );
phi3                    = pi / 2 - phi1 - phi2;
Dx                      = r - DF * sin(phi3);
Dy                      = h + DF * cos(phi3);

Dy1                     = Dy - (AB - AC);
phi4                    = atan( (r - Dx) / (Dy - h + gripper_height) );
Ex                      = r - EF * sin(phi4);
Ey                      = EF * cos(phi4) - h;

Ey1                     = Ey - AB;
BE                      = sqrt( Ex^2 + Ey1^2 );

Z                       = pi / 2 - acos( Dx / CD );
X                       = pi / 2 - atan( Ey1 / Ex ) - acos( (BG^2 + BE^2 - GE^2) / 2 / BG / BE);

%%
%Check if constraints are met

boundsR                 = (R > (-1 * pi / 2) && R < (pi / 2));
boundsZ                 = (Z > -0.4 && Z < 0.3);
boundsX                 = (X > 0.2 && X < 1.47);

if (boundsR)
    ReferenceR = R;
else
    error("Reference angle R is out of bounds")
end

if (boundsX)
    ReferenceX = X;
else
    error("Reference angle X is out of bounds")
end

if (boundsZ)
    ReferenceZ = Z;
else
    error("Reference angle Z is out of bounds")
end

%%
%Run the model: (NEEDS TO BE AT THE END OF THE SCRIPT)
open_system('RobotArm')
model_workspace = get_param('RobotArm','ModelWorkspace');
sim('RobotArm.slx')