% MAKE SURE YOU SET THE VALUES FOR THE REFERENCE INPUTS (R, Z, X) TO THE
% VARIABLES 'ReferenceR', 'ReferenceZ' AND 'ReferenceX' IN THE SIMULINK
% FILE.

%%
%Input
%See the image for a definition of the coordinate systems

x                       = -5; %[m] x postition as defined in figure
y                       = 5; %[m] y postition as defined in figure
robot_to_conveyorbelt   = 0; %[m] distance from the robot to the conveyorbelt
h                       = 0; %[m] desired height

%%
%Conversion from (x,y) to (R,r)

y                       = y + robot_to_conveyorbelt; %Shifting origin
r                       = sqrt( x * x + y * y ); %Pythagorean theorem
R                       = atan( x / y ); %Standard arctangent rule

%%
%Conversion from (r,h) to (X,Z)
X=0.3;
Z=0;
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