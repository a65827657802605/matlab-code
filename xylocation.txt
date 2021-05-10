%theta1 and theta2 are defined as the angle above the horizontal, with
%theta1 the angle at B and theta2 the angle at C

%F_x is the x position of F
%F_y is the y position of F
%other points are labelled similarly.

%theta_2_min = -0.4;
%theta_2_max = 0.3;
%theta_1_min = 0.2;
%theta_1_max = 1.47;

function [] = xylocation(theta1,theta2)
    
%given lengths
DF = 185;
CD = 130;
AB = 81;
BC = 165;
GE = 130;
BG = 80;
DE = 120;
EF = 65;
B_x = 0;
B_y = AB;
A_x = 0;
A_y = 0;
C_x = 0;
C_y = AB + BC;

theta_2_min = -0.4;
theta_2_max = 0.3;
theta_1_min = 0.2;
theta_1_max = 1.47;

%first find the postion of F, given theta1 and theta2

G_x = BG * cos(theta1); %x position of G
G_y = AB + BG * sin(theta1); %y position of G
D_x = CD * cos(theta2); %x position of D
D_y1 = CD * sin(theta2); %height of D above AB + BC
D_y = AB + BC + D_y1; %y position of D

DG = sqrt((D_x - G_x)^2 + (D_y - G_y)^2); %line between DG is hypotenuse of triangle with other two sides parallel to x,y directions
gamma_1 = acos((DG^2 +GE^2 -DE^2)/(2 * DG * GE)); %angle at point G, between line GD and line GE
gamma_2 = atan((D_x-G_x)/(D_y-G_y)); %angle at point G, between line DG and horizontal line projected from point G
gamma_3 = pi/2 - gamma_1- gamma_2; %angle between the horizontal line projected from G and the line GE, whether positive or negative

E_x1 = GE * cos(gamma_3); %x distance of E past G
E_y1 = GE * sin(gamma_3);%height of E above G
E_x = G_x +E_x1; %x position of E
E_y = G_y + E_y1;%y position of E

gamma_4 = asin((E_x - D_x)/DE); %angle between line DE and vertical line projected downwards from point D

F_x = D_x + DF * sin(gamma_4); % y location of F
F_y = D_y - DF * cos(gamma_4); % x location of F

F_x
F_y

xvalues = [A_x, B_x, C_x, D_x, E_x, F_x, E_x, G_x, B_x];
yvalues = [A_y, B_y, C_y, D_y, E_y, F_y, E_y, G_y, B_y];

figure(1); %plot from the anges given to the x,y loctation
plot(xvalues,yvalues,'-x')
title('robot arm configuration based on given theta1 and theta2')

clear G_x G_y D_x D_y1 D_y E_x1 E_y1 E_x E_y  %clear all previously determined locations of points G,D,E




%part two
%given F_x and F_y, calculate what theta1 and theta2 are

CF = sqrt(F_x^2 + (AB + BC - F_y)^2); %length of line between C and F
phi_1 = acos(F_x / CF); %angle of right angled triangle C-F-point on vertical line AB, between sides CF and AB vertical line)
phi_2 = acos((CF^2 + DF^2 - CD^2)/ (2 * CF * DF)); %angle between CF and DF, triangle CDF
phi_3 = pi/2 - phi_1 - phi_2; %angle between vertical line projected upwards from point F,and line DF
D_x = F_x - DF * sin(phi_3); %F_x position of D
D_y = F_y + DF * cos(phi_3); %y position of D

%calculations for theta1, given location of D
D_y1 = D_y - (AB + BC); %difference in height between C and D
theta2calculated = asin(D_y1/CD);  %different name for theta so matlab doesn't get confused with the variable defined earlier


%calculations for position of E, given location of D and F

phi_4 = atan((F_x-D_x)/(D_y - F_y)); %angle between DF and line projected vertically downwards from point D
E_x = F_x - EF * sin(phi_4); %F_x location of E
E_y = F_y + EF * cos(phi_4); %y location of E

%calculations for theta1, given location of point E
E_y1 = E_y - AB; %height of E above B
BE = sqrt(E_x^2 + E_y1^2); %length of line BE
phi_5 = atan(E_y1/E_x); %angle between BE and horizontal line projected from B
phi_6 = acos((BG^2 + BE^2 - GE^2)/(2 * BG * BE)); %angle between BE and BG
theta1calculated= phi_5 + phi_6; 

theta1calculated
theta2calculated

G_x = BG * cos(theta1calculated);
G_y = AB + BG * sin(theta1calculated);

xvaluescalculated = [A_x, B_x, C_x, D_x, E_x, F_x, E_x, G_x, B_x];
yvaluescalculated = [A_y, B_y, C_y, D_y, E_y, F_y, E_y, G_y, B_y];


figure(2); %plot from x,y given back to angles given
plot(xvaluescalculated,yvaluescalculated,'-x')
title('robot arm configuration based on given x,y location (calculated from the original given angles)') 

end 













