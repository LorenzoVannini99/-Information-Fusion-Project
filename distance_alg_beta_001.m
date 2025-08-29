%The main idea is to create maps and a sensor fusion algorithm
%Every robot explore a certain part of the map and every robot has a
%certain degree of confidence about the map is looking
%Combining the information of the robots in order to create a sort of "shades fo confidence" where black is certain and white is 0 confidence area 
%Load a set of example binary occupancy grids from exampleMaps, including simpleMap, which this example uses.
load exampleMaps.mat

%Creates an occupancy map with a specified grid resolution in cells per
%meter,using simpleMap with a resolution of 1
refMap = occupancyMap(simpleMap,1);
refFigure = figure('Name','SimpleMap');
show(refMap);

%_______________________________________________________________________________________________________________________________________________________________
%Robot 1

%Creates an occupancy map with a specified grid resolution in cells per
%meter,using simpleMap with a resolution of 10
[mapdimx,mapdimy] = size(simpleMap);
map = occupancyMap(mapdimy,mapdimx,10);
mapFigure = figure('Name','Live Unknown Map robot 1');
show(map);

%Initialize Motion Model and Controller
%Create a differential-drive kinematic motion model. 
%The motion model represents the motion of the simulated differential-drive robot.
%This model takes left and right wheels speeds or linear and angular velocities for the robot heading.
diffDrive = differentialDriveKinematics("VehicleInputs","VehicleSpeedHeadingRate");

%Create a pure pursuit controller. This controller generates the velocity inputs for the simulated robot to follow a desired path. 
%Set your desired linear velocity and maximum angular velocity, specified in meters per second and radians per second respectively.
controller = controllerPurePursuit('DesiredLinearVelocity',2,'MaxAngularVelocity',2);

%Set Up Range Sensor
%Create a sensor with a max range maxrange
%This sensor simulates range readings based on a given pose and map. 
%The reference map is used with this range sensor to simulate collecting sensor readings in an unknown environment.
maxrange=6;
sensor = rangeSensor;
sensor.Range = [0,maxrange];

%Create the Planned Path
%Create a path to drive through the map for gathering range sensor readings.
path = [4 6; 6.5 12.5 ; 3 17 ; 6 22 ; 10 23 ];

%To be as realistic as possibile we xan add an efficency constant based on
%the length of the path
d_1=0;
for i=1:(length(path)-1)
    
v=path(i+1,:)-path(i,:);

d_1=sum(v.^2)+d_1;

end

d_f_1=sqrt(d_1)

%Creating a function that measure the efficency decade
k1=1;
alpha1=0.001;
eta_1=k1*exp(-alpha1*d_f_1)

%Plot the path on the reference map figure.
figure(refFigure);
hold on
plot(path(:,1),path(:,2), 'o-');
hold off

%Set the path as the waypoints of the pure pursuit controller.
controller.Waypoints = path;

%Follow Path and Map Environment
%Set the initial pose and final goal location based on the path. Create global variables for storing the current pose and an index for tracking the iterations.
initPose = [path(1,1) path(1,2), pi/2];
goal = [path(end,1) path(end,2)]';
poses(:,1) = (initPose)';

%The exampleHelperDiffDriveControl function has the following workflow:

% Scan the reference map using the range sensor and the current pose. This simulates normal range readings for driving in an unknown environment.
% Update the map with the range readings.
% Get control commands from pure pursuit controller to drive to next waypoint.
% Calculate derivative of robot motion based on control commands
% Increment the robot pose based on the derivative.
% You should see the robot driving around the empty map and filling in walls as the range sensor detects them.
exampleHelperDiffDriveCtrl(diffDrive,controller,initPose,goal,refMap,map,refFigure,mapFigure,sensor)

%_______________________________________________________________________________________________________________________________________________________________
%Robot 2

%Create an empty map of the same dimensions as the selected map with a resolution of 10. Show the figure and save the handle of the figure. 
%Lock the axes at the size of the map.
[mapdimx_2,mapdimy_2] = size(simpleMap);
map_2= occupancyMap(mapdimy_2,mapdimx_2,10);
mapFigure = figure('Name','Live Unknown Map robot 2');
show(map_2);

%Initialize Motion Model and Controller
%Create a differential-drive kinematic motion model. The motion model represents the motion of the simulated differential-drive robot.
%This model takes left and right wheels speeds or linear and angular velocities for the robot heading. 
%For this example, use the vehicle speed and heading rate for the VehicleInputs.
diffDrive_2= differentialDriveKinematics("VehicleInputs","VehicleSpeedHeadingRate");

%Create a pure pursuit controller. This controller generates the velocity inputs for the simulated robot to follow a desired path. Set your desired linear velocity and maximum angular velocity, specified in meters per second and radians per second respectively.
controller_2= controllerPurePursuit('DesiredLinearVelocity',2,'MaxAngularVelocity',2);

%Set Up Range Sensor
%Create a sensor with a max range maxrange
%This sensor simulates range readings based on a given pose and map. 
%The reference map is used with this range sensor to simulate collecting sensor readings in an unknown environment.
maxrange_2=5;
sensor_2= rangeSensor;
sensor_2.Range= [0,maxrange_2];

%Create the Planned Path
%Create a path to drive through the map for gathering range sensor readings.
path_2= [4 4; 10 15; 14 16; 16 20; 22 22 ; 24 16];

%To be as realistic as possibile we can add an efficency constant based on
%the length of the path
%Calculating the distance of the path 2
d_2=0;
for i=1:(length(path_2)-1)   
v=path_2(i+1,:)-path_2(i,:);
d_2=sum(v.^2)+d_2;

end
d_f_2=sqrt(d_2)

%Creating a function that measure the efficency decade
k2=1;
alpha2=0.001;
eta_2=k1*exp(-alpha2*d_f_2)


%Plot the path on the reference map figure.
figure(refFigure);
hold on
plot(path_2(:,1),path_2(:,2), 'o-');
hold off

%Set the path as the waypoints of the pure pursuit controller.
controller_2.Waypoints = path_2;

%Follow Path and Map Environment
%Set the initial pose and final goal location based on the path. Create global variables for storing the current pose and an index for tracking the iterations.
initPose_2= [path_2(1,1) path_2(1,2), pi/2];
goal_2 = [path_2(end,1) path_2(end,2)]';
poses_2(:,1) = (initPose_2)';

%The exampleHelperDiffDriveControl function has the following workflow:
% Scan the reference map using the range sensor and the current pose. This simulates normal range readings for driving in an unknown environment.
% Update the map with the range readings.
% Get control commands from pure pursuit controller to drive to next waypoint.
% Calculate derivative of robot motion based on control commands
% Increment the robot pose based on the derivative.
% You should see the robot driving around the empty map and filling in walls as the range sensor detects them.
exampleHelperDiffDriveCtrl(diffDrive_2,controller_2,initPose_2,goal_2,refMap,map_2,refFigure,mapFigure,sensor_2)


figure
title('Second Partial Map')
show(map_2)

%_______________________________________________________________________________________________________________________________________________________________
%Robot 3

%Create an empty map of the same dimensions as the selected map with a resolution of 10. 
%Lock the axes at the size of the map.
[mapdimx_3,mapdimy_3] = size(simpleMap);
map_3= occupancyMap(mapdimy_3,mapdimx_3,10);
mapFigure = figure('Name','Live Unknown Map robot 3');
show(map_3);

%Initialize Motion Model and Controllernn
%Create a differential-drive kinematic motion model. The motion model represents the motion of the simulated differential-drive robot.
%This model takes left and right wheels speeds or linear and angular velocities for the robot heading. 
%For this example, use the vehicle speed and heading rate for the VehicleInputs.
diffDrive_3= differentialDriveKinematics("VehicleInputs","VehicleSpeedHeadingRate");

%Create a pure pursuit controller. This controller generates the velocity inputs for the simulated robot to follow a desired path. Set your desired linear velocity and maximum angular velocity, specified in meters per second and radians per second respectively.
controller_3= controllerPurePursuit('DesiredLinearVelocity',1.8,'MaxAngularVelocity',1.8);

%Set Up Range Sensor
%Create a sensor with a max range maxrange
%This sensor simulates range readings based on a given pose and map. 
%The reference map is used with this range sensor to simulate collecting sensor readings in an unknown environment.
maxrange_3=5;
sensor_3= rangeSensor;
sensor_3.Range= [0,maxrange_3];

%Create the Planned Path
%Create a path to drive through the map for gathering range sensor readings.
path_3= [3 3; 11 15; 16 15; 20 8; 15 3 ; 24 3];

%To be as realistic as possibile we xan add an efficency constant based on
%the length of the path
%Calculating the distance of the path
d_3=0;
for i=1:(length(path)-1)
    
v=path_3(i+1,:)-path_3(i,:);

d_3=sum(v.^2)+d_3;

end

d_f_3=sqrt(d_3)

%Creating a function that measure the efficency decade
k3=1;
alpha3=0.001;
eta_3=k3*exp(-alpha3*d_f_3)


%Plot the path on the reference map figure.
figure(refFigure);
hold on
plot(path_3(:,1),path_3(:,2), 'o-');
hold off

%Set the path as the waypoints of the pure pursuit controller.
controller_3.Waypoints = path_3;

%Follow Path and Map Environment
%Set the initial pose and final goal location based on the path. Create global variables for storing the current pose and an index for tracking the iterations.
initPose_3= [path_3(1,1) path_3(1,2), pi/2];
goal_3 = [path_3(end,1) path_3(end,2)]';
poses_3(:,1) = (initPose_3)';

%The exampleHelperDiffDriveControl function has the following workflow:
% Scan the reference map using the range sensor and the current pose. This simulates normal range readings for driving in an unknown environment.
% Update the map with the range readings.
% Get control commands from pure pursuit controller to drive to next waypoint.
% Calculate derivative of robot motion based on control commands
% Increment the robot pose based on the derivative.
% You should see the robot driving around the empty map and filling in walls as the range sensor detects them.
exampleHelperDiffDriveCtrl(diffDrive_3,controller_3,initPose_3,goal_3,refMap,map_3,refFigure,mapFigure,sensor_3)

%____________________________________________________________________________________________________________________________________________________________________
%ADDING REALISTIC SENSOR NOISE TO THE SYSTEM (1 robot)
%We can add a threshold if a value is greater than 0.95 set it to 1 and if
%less than 0.15 set it to 0

%Convert occupancy map to matrix
%mat = occupancyMatrix(map) returns probability values stored in the occupancy grid object as a matrix.
%map = binaryOccupancyMap(p)
matrix_map=occupancyMatrix(map);

%We might add to the system a white Gaussian noise to simulate the possible
%uncertainty of the robot
dim_map=size(matrix_map);
dim_map_rows=dim_map(1);
dim_map_columns=dim_map(2);

matrix_map_unc=zeros(dim_map_rows,dim_map_columns);

noise_variance=0.05;

for i=1:(dim_map_rows)
    for j=1:(dim_map_columns)
        
     random_matrix=noise_variance*randn(dim_map_rows,dim_map_columns);
     matrix_map_unc(i,j)=matrix_map(i,j)+random_matrix(i,j);
     
    end   
end
    
    
 for i=1:(dim_map_rows)
    for j=1:(dim_map_columns)
     
        if (matrix_map(i,j)<0.51)&&(matrix_map(i,j)>0.49)
            matrix_map_unc(i,j)=0.5;
        end
            
       if (matrix_map_unc(i,j)>0.95)
            matrix_map_unc(i,j)=1;
        end
        
         if (matrix_map_unc(i,j)<0.1)
             matrix_map_unc(i,j)=0;
         end
    end  
    
 end
 
 
%Creating the first map with noise
map_unc_1=occupancyMap(matrix_map_unc);
show(map_unc_1)


%ADDING REALISTIC SENSOR NOISE TO THE SYSTEM (2 robot)
%Convert occupancy map to matrix
%mat = occupancyMatrix(map) returns probability values stored in the occupancy grid object as a matrix.
%map = binaryOccupancyMap(p)
matrix_map_2=occupancyMatrix(map_2);

%We might add to the system a noise to simulate the uncertainty of the robot
dim_map=size(matrix_map_2);
dim_map_rows=dim_map(1);
dim_map_columns=dim_map(2);

matrix_map_unc_2=zeros(dim_map_rows,dim_map_columns);

for i=1:(dim_map_rows)
    for j=1:(dim_map_columns)
        
     random_matrix=noise_variance*randn(dim_map_rows,dim_map_columns);
     matrix_map_unc_2(i,j)=matrix_map_2(i,j)+random_matrix(i,j);
     
    end   
end
    
    
 for i=1:(dim_map_rows)
    for j=1:(dim_map_columns)
     
        if (matrix_map_2(i,j)<0.51)&&(matrix_map_2(i,j)>0.49)
            matrix_map_unc_2(i,j)=0.5;
        end
            
       if (matrix_map_unc_2(i,j)>0.95)
            matrix_map_unc_2(i,j)=1;
        end
        
         if (matrix_map_unc_2(i,j)<0.1)
             matrix_map_unc_2(i,j)=0;
         end
    end  
    
 end
 
%Creating the second map with noise
map_unc_2=occupancyMap(matrix_map_unc_2);
show(map_unc_2)

%ADDING REALISTIC SENSOR NOISE TO THE SYSTEM (3 robot)

%Convert occupancy map to matrix
%mat = occupancyMatrix(map) returns probability values stored in the occupancy grid object as a matrix.
%map = binaryOccupancyMap(p)
matrix_map_3=occupancyMatrix(map_3);

%We might add to the system a noise to simulate the uncertainty of the robot
dim_map=size(matrix_map_3);
dim_map_rows=dim_map(1);
dim_map_columns=dim_map(2);

matrix_map_unc_3=zeros(dim_map_rows,dim_map_columns);

for i=1:(dim_map_rows)
    for j=1:(dim_map_columns)
        
     random_matrix=noise_variance*randn(dim_map_rows,dim_map_columns);
     matrix_map_unc_3(i,j)=matrix_map_3(i,j)+random_matrix(i,j);
     
    end   
end
    
    
 for i=1:(dim_map_rows)
    for j=1:(dim_map_columns)
     
        if (matrix_map_3(i,j)<0.51)&&(matrix_map_3(i,j)>0.49)
            matrix_map_unc_3(i,j)=0.5;
        end
            
       if (matrix_map_unc_3(i,j)>0.95)
            matrix_map_unc_3(i,j)=1;
        end
        
         if (matrix_map_unc_3(i,j)<0.1)
             matrix_map_unc_3(i,j)=0;
         end
    end  
    
 end
 
%Creating the second map with noise
map_unc_3=occupancyMap(matrix_map_unc_3);
show(map_unc_3)



%_______________________________________________________________________________________________________________________________________________________________
%SENSOR FUSION
%Using the travelled distance algorithm

%path 1
path = [200  50; 6.5 12.5 ; 3 17 ; 6 22 ; 10 23 ];

%path 2
path_2= [210 40; 10 15; 14 16; 16 20; 22 22 ; 24 16];

%path 3
path_3= [190 60; 11 15; 16 15; 20 8; 15 3 ; 24 3];

%defining the beta coefficent
beta=0.01

%Defining the distance matrixit's a n x m x 3 matrix where in the
%ij column i have a 3-vector containing the efficency (eta1,eta2,eta3)
Distance_matrix=zeros(dim_map_rows,dim_map_columns,3);

%Defining the efficency distance value it's a n x m x 3 matrix where in the
%ij column i have a 3-vector containing the efficency (eta1,eta2,eta3)
Eta_matrix=zeros(dim_map_rows,dim_map_columns,3);

for i=1:(dim_map_rows)
    for j=1:(dim_map_columns)
        
        %position of the i j th cell
        p=[i,j];
         
        Distance_matrix(i,j,1)=norm(p- path(1,:));
        Distance_matrix(i,j,2)=norm(p- path_2(1,:));
        Distance_matrix(i,j,3)=norm(p- path_3(1,:));
        
        for r=1:3
        Eta_matrix(i,j,r)=exp(-beta*Distance_matrix(i,j,r));
        end
    end
end



fused_matrix=zeros(dim_map_rows,dim_map_columns);

 for i=1:(dim_map_rows)
    for j=1:(dim_map_columns)
        
        
         %adding uncertainty distance
         matrix_map_unc(i,j)=0.5+(matrix_map_unc(i,j)-0.5)*Eta_matrix(i,j,1);
         matrix_map_unc_2(i,j)=0.5+(matrix_map_unc_2(i,j)-0.5)*Eta_matrix(i,j,2);
         matrix_map_unc_3(i,j)=0.5+(matrix_map_unc_3(i,j)-0.5)*Eta_matrix(i,j,3);
   
        %adding uncertainty for the overall distance travelled
        matrix_map_unc(i,j)=0.5+(matrix_map_unc(i,j)-0.5)*eta_1;
        matrix_map_unc_2(i,j)=0.5+(matrix_map_unc_2(i,j)-0.5)*eta_2;
        matrix_map_unc_3(i,j)=0.5+(matrix_map_unc_3(i,j)-0.5)*eta_3;
        
        num=( matrix_map_unc_3(i,j) * matrix_map_unc_2(i,j) * matrix_map_unc(i,j) )^(0.3333);
        den=( matrix_map_unc_3(i,j) * matrix_map_unc_2(i,j) * matrix_map_unc(i,j) )^(0.33333) + ( ( 1-matrix_map_unc_3(i,j) ) * ( 1-matrix_map_unc_2(i,j) ) * ( 1-matrix_map_unc(i,j) ) )^(0.33333);
        fused_matrix(i,j)=num/den;
          
    end
 end
 
%Fused map 
fused_matrix(isnan(fused_matrix))=0;
fused_map=occupancyMap(fused_matrix,1);
figure
show(fused_map)


%updateOccupancy(fused_map,[x y],pvalues)
%updateOccupancy(fused_map,[50,200],1)
%show(fused_map)
%map = occupancyMap(10,10,10);
%ij = world2grid(map,[x y]);
%xy = grid2local(map,ij)
