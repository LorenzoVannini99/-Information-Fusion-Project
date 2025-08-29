%The main idea is to create maps and a sensor fusion algorithm
%Every robot explore a certain part of the map and every robot has a
%certain degree of confidence about the map is looking
%Combining the information of the robots in order to create a sort of "shades fo confidence" where black is certain and white is 0 confidence area 


%Load a set of example binary occupancy grids from exampleMaps, including simpleMap, which this example uses.
load exampleMaps.mat

%Create the reference binary occupancy map using simpleMap with a resolution of 1. Show the figure and save the handle of the figure.
refMap = occupancyMap(simpleMap,1);
refFigure = figure('Name','SimpleMap');
show(refMap);

%_______________________________________________________________________________________________________________________________________________________________
%Robot 1

%Create an empty map of the same dimensions as the selected map with a resolution of 10. Show the figure and save the handle of the figure. 
%Lock the axes at the size of the map.
[mapdimx,mapdimy] = size(simpleMap);
map = occupancyMap(mapdimy,mapdimx,10);
mapFigure = figure('Name','Live Unknown Map robot 1');
show(map);

%Initialize Motion Model and Controller
%Create a differential-drive kinematic motion model. The motion model represents the motion of the simulated differential-drive robot.
%This model takes left and right wheels speeds or linear and angular velocities for the robot heading. 
%For this example, use the vehicle speed and heading rate for the VehicleInputs.
diffDrive = differentialDriveKinematics("VehicleInputs","VehicleSpeedHeadingRate");

%Create a pure pursuit controller. This controller generates the velocity inputs for the simulated robot to follow a desired path. 
%Set your desired linear velocity and maximum angular velocity, specified in meters per second and radians per second respectively.
controller = controllerPurePursuit('DesiredLinearVelocity',2.6,'MaxAngularVelocity',2.6);

%Set Up Range Sensor
%Create a sensor with a max range maxrange
%This sensor simulates range readings based on a given pose and map. 
%The reference map is used with this range sensor to simulate collecting sensor readings in an unknown environment.
maxrange=4;
sensor = rangeSensor;
sensor.Range = [0,maxrange];

%Create the Planned Path
%Create a path to drive through the map for gathering range sensor readings.
path = [4 6; 6.5 12.5 ; 3 17 ; 6 22 ; 10 23 ];

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
controller_2= controllerPurePursuit('DesiredLinearVelocity',2.7,'MaxAngularVelocity',2.7);

%Set Up Range Sensor
%Create a sensor with a max range maxrange
%This sensor simulates range readings based on a given pose and map. 
%The reference map is used with this range sensor to simulate collecting sensor readings in an unknown environment.
maxrange_2=4;
sensor_2= rangeSensor;
sensor_2.Range= [0,maxrange_2];

%Create the Planned Path
%Create a path to drive through the map for gathering range sensor readings.
path_2= [4 4; 10 15; 14 16; 16 20; 22 22 ; 24 16];

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
title('First Partial Map')
show(map)

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
maxrange_3=4;
sensor_3= rangeSensor;
sensor_3.Range= [0,maxrange_3];

%Create the Planned Path
%Create a path to drive through the map for gathering range sensor readings.
path_3= [3 3; 11 15; 16 15; 20 8; 15 3 ; 24 3];

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

for i=1:(dim_map_rows)
    for j=1:(dim_map_columns)
        
     random_matrix=0.1*rand(dim_map_rows,dim_map_columns);
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
        
     random_matrix=0.1*rand(dim_map_rows,dim_map_columns);
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
        
     random_matrix=0.1*rand(dim_map_rows,dim_map_columns);
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
%Using a sensor fusion algorithm
fused_matrix=zeros(size(map_unc_2));

 for i=1:(dim_map_rows)
    for j=1:(dim_map_columns)
        
        num=( matrix_map_unc_3(i,j) * matrix_map_unc_2(i,j) * matrix_map_unc(i,j) )^(0.333);
        
        den=( matrix_map_unc_3(i,j) * matrix_map_unc_2(i,j) * matrix_map_unc(i,j) )^(0.333) + ( ( 1-matrix_map_unc_3(i,j) ) * ( 1-matrix_map_unc_2(i,j) ) * ( 1-matrix_map_unc(i,j) ) )^(0.333);
        fused_matrix(i,j)=num/den;
    end
 end
 
%Fused map 
fused_matrix(isnan(fused_matrix))=0;
fused_map=occupancyMap(fused_matrix);
show(fused_map)
 

