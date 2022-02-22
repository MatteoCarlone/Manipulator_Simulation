%% MOCOM_LAB1 Matteo Carlone s4652067 , Luca Predieri s4667708.

clc
clear
close all

% adding the path to the folder that contains the functions.
addpath('include');

% Initialization of the transformation matrix from link i to link i+1 for
% qi=0.

biTri(1,1,1) = 1; biTri(1,2,1) = 0; biTri(1,3,1) = 0; biTri(1,4,1) = 0;
biTri(2,1,1) = 0; biTri(2,2,1) = 1; biTri(2,3,1) = 0; biTri(2,4,1) = 0;
biTri(3,1,1) = 0; biTri(3,2,1) = 0; biTri(3,3,1) = 1; biTri(3,4,1) = 0.175;
biTri(4,1,1) = 0; biTri(4,2,1) = 0; biTri(4,3,1) = 0; biTri(4,4,1) = 1;

biTri(1,1,2) = -1; biTri(1,2,2) = 0;  biTri(1,3,2) = 0; biTri(1,4,2) = 0;
biTri(2,1,2) = 0;  biTri(2,2,2) = 0;  biTri(2,3,2) = 1; biTri(2,4,2) = 0;
biTri(3,1,2) = 0;  biTri(3,2,2) = 1;  biTri(3,3,2) = 0; biTri(3,4,2) = 0.108;
biTri(4,1,2) = 0;  biTri(4,2,2) = 0;  biTri(4,3,2) = 0; biTri(4,4,2) = 1;

biTri(1,1,3) = 0;  biTri(1,2,3) = 0; biTri(1,3,3) = 1;  biTri(1,4,3) = 0.105;
biTri(2,1,3) = -1; biTri(2,2,3) = 0; biTri(2,3,3) = 0;  biTri(2,4,3) = 0;
biTri(3,1,3) = 0;  biTri(3,2,3) = -1; biTri(3,3,3) = 0; biTri(3,4,3) = 0;
biTri(4,1,3) = 0;  biTri(4,2,3) = 0; biTri(4,3,3) = 0;  biTri(4,4,3) = 1;

biTri(1,1,4) = 0;  biTri(1,2,4) = 1;  biTri(1,3,4) = 0;  biTri(1,4,4) = -0.1455;
biTri(2,1,4) = 0;  biTri(2,2,4) = 0;  biTri(2,3,4) = -1; biTri(2,4,4) = 0;
biTri(3,1,4) = -1; biTri(3,2,4) = 0;  biTri(3,3,4) = 0;  biTri(3,4,4) = 0.3265;
biTri(4,1,4) = 0;  biTri(4,2,4) = 0;  biTri(4,3,4) = 0;  biTri(4,4,4) = 1;

biTri(1,1,5) = 0; biTri(1,2,5) = 0;  biTri(1,3,5) = 1; biTri(1,4,5) = 0.095;
biTri(2,1,5) = 0; biTri(2,2,5) = -1; biTri(2,3,5) = 0; biTri(2,4,5) = 0;
biTri(3,1,5) = 1; biTri(3,2,5) = 0;  biTri(3,3,5) = 0; biTri(3,4,5) = 0;
biTri(4,1,5) = 0; biTri(4,2,5) = 0;  biTri(4,3,5) = 0; biTri(4,4,5) = 1;

biTri(1,1,6) = 0; biTri(1,2,6) = 0;  biTri(1,3,6) = 1; biTri(1,4,6) = 0;
biTri(2,1,6) = 0; biTri(2,2,6) = -1; biTri(2,3,6) = 0; biTri(2,4,6) = 0;
biTri(3,1,6) = 1; biTri(3,2,6) = 0;  biTri(3,3,6) = 0; biTri(3,4,6) = 0.325;
biTri(4,1,6) = 0; biTri(4,2,6) = 0;  biTri(4,3,6) = 0; biTri(4,4,6) = 1;

biTri(1,1,7) = 0; biTri(1,2,7) = 0;  biTri(1,3,7) = 1; biTri(1,4,7) = 0.132;
biTri(2,1,7) = 0; biTri(2,2,7) = -1; biTri(2,3,7) = 0; biTri(2,4,7) = 0;
biTri(3,1,7) = 1; biTri(3,2,7) = 0;  biTri(3,3,7) = 0; biTri(3,4,7) = 0;
biTri(4,1,7) = 0; biTri(4,2,7) = 0;  biTri(4,3,7) = 0; biTri(4,4,7) = 1;

% Number of links of the manipulator.

linkNumber = size(biTri,3);

% Defining the matrix of the type of each link, by default 0 because our
% manipulator has only revolute joints.

linkType = zeros(linkNumber,1);

% Matrix which contains the basic vectors between joints.

bri = zeros(3,linkNumber);

% Matrix which contains all the transformation matrix for each link with 
% respect to base, the return of GetTransformationWrtBase() function.

bTi = zeros(4,4, linkNumber);

% Initialization of q, the position of the current links.

q = [0.5,1,0.5,1,0.5,1,0.5];

% Calling GetDirectGeometry(), which returns all the model matrices given
% a certain q configuration.

biTei = GetDirectGeometry(q, biTri, linkType);

% Calling with a for GetTransformationWrtBase().

for i = 1:linkNumber
    
    bTi(:,:,i)=GetTransformationWrtBase(biTei,i);

end 

%-------------------MOVE----------------------%

% Initialization of the initial and final position.

initial_position=[0,0,0,0,0,0,0];
final_position=[2*pi,pi/2,0,pi,0,2*pi,0];

% Defining number of steps in the plot.

StepsNumber=65;

% For each single link we want to separate the path in n steps.

qSteps=[linspace(initial_position(1),final_position(1),StepsNumber)',...
    linspace(initial_position(2),final_position(2),StepsNumber)',...
    linspace(initial_position(3),final_position(3),StepsNumber)',...
    linspace(initial_position(4),final_position(4),StepsNumber)',...
    linspace(initial_position(5),final_position(5),StepsNumber)',...
    linspace(initial_position(6),final_position(6),StepsNumber)',...
    linspace(initial_position(7),final_position(7),StepsNumber)'];

% Plotting the movement of the manipulator.

figure

hold on
xlabel('x')
ylabel('y')
zlabel('z')
grid on
axis equal
azimuth = 50;
elevation = 25;

% Defining the point of view of the view.

view(azimuth,elevation)

% Plotting all the frames through a for loop.

for i = 1:StepsNumber
    
    % Initialazing the vector matrix including the base (0; 0; 0).

    brij=zeros(3,linkNumber+1); 

    q = qSteps(i,1:linkNumber)';

    % Filling the bitei matrix using the function GetDirectGeometry().
    
    biTei = GetDirectGeometry(q,biTri,linkType);
    
    % Extrapolating basic vector with respect to the base from biTei.
    % taking into account the presence of the base (0,0,0).

    for j = 1:linkNumber
        
        brij(:,j+1) = GetBasicVectorWrtBase(biTei,j);

    end
    
    % plotting the joints

    for j = 2:linkNumber+1
        
       plot3(brij(1,j),brij(2,j),brij(3,j),'k.', 'MarkerSize', 15)
       
    end

    % plotting the lines connecting all the joints. 
    
    line(brij(1,:),brij(2,:),brij(3,:),'LineWidth',3,'Color', '#4DBEEE')

    % plotting the base in (0,0,0).
    
    plot3(0,0,0,'red.','MarkerSize', 30)

    hold on

    getframe;
    
    % refreshing the window every loop-1.
    if i<StepsNumber 
        % function that clean the window.
        cla();
    end

end





