% © Rahul Kala, IIIT Allahabad, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 
% Please cite the work in all materials as: R. Kala (2014) Code for Robot Path Planning using Fuzzy Logic, Indian Institute of Information Technology Allahabad, Available at: http://rkala.in/codes.html

map=int16(im2bw(imread('map1.bmp'))); % input map read from a bmp file. for new maps write the file name here
source=[20 20]; % source position in Y, X format
goal=[480 480]; % goal position in Y, X format
robotDirection=pi/4; % initial heading direction
robotSize=[10 10]; %length and breadth
robotSpeed=10; % arbitrary units 
maxRobotSpeed=10; % arbitrary units 
S=10; % safety distance
distanceThreshold=30; % a threshold distace. points within this threshold can be taken as same. 
maxAcceleration=10; % maximum speed change per unit time
directionScaling=60*pi/180; % fuzzy outputs to turn are restriect to -1 and 1. these are magnified here. maximum turn can be 60 degrees

%%%%% parameters end here %%%%%

fuz=readfis('fuzzyBase.fis'); % fuzzy inference system used. to read/edit use fuzzy(readfis('fuzzyBase.fis')) at the command line
distanceScaling=(size(map,1)^2+size(map,2)^2)^0.5; % all inputs are scaled by this number so that all distance inputs are between 0 and 1. maximum distance can be distanceScaling
currentPosition=source; % position of the centre of the robot
currentDirection=robotDirection; % direction of orientation of the robot
robotHalfDiagonalDistance=((robotSize(1)/2)^2+(robotSize(2)/2)^2)^0.5; % used for distance calculations 
pathFound=false; % has goal been reached
prevTurn=0; % preffered turn at the previous time step, used for turning heuristic, see variable turn being set below.
prevDistanceLeftDiagonal=distanceScaling; % diagonal distance at the previous time step, used for tracking obstacles, used for turning heuristic, see variable turn being set below. 
prevDistanceRightDiagonal=distanceScaling; % diagonal distance at the previous time step, used for tracking obstacles, used for turning heuristic, see variable turn being set below. 
pathCost=0;
t=1;
imshow(map==1);
rectangle('position',[1 1 size(map)-1],'edgecolor','k');
pathLength=0; 
if ~plotRobot(currentPosition,currentDirection,map,robotHalfDiagonalDistance)
     error('source lies on an obstacle or outside map'); 
end
M(t)=getframe;
t=t+1;

if ~feasiblePoint(goal,map), error('goal lies on an obstacle or outside map'); end

tic;
while ~pathFound
    
    % calculate distance from obstacle at front
    for i=robotSize(1)/2+1:distanceScaling
        x=int16(currentPosition+i*[sin(currentDirection) cos(currentDirection)]);
        if ~feasiblePoint(x,map), break; end
    end
    distanceFront=(i-robotSize(1)/2)/distanceScaling; % robotSize(1)/2 distance included in i was inside the robot body 
    
    % calculate distance from obstacle at front-left diagonal
    for i=robotHalfDiagonalDistance+1:distanceScaling
        x=int16(currentPosition+i*[sin(currentDirection-pi/4) cos(currentDirection-pi/4)]);
        if ~feasiblePoint(x,map), break; end
    end
    distanceFrontLeftDiagonal=(i-robotHalfDiagonalDistance)/distanceScaling;
    
    % calculate distance from obstacle at front-right diagonal
    for i=robotHalfDiagonalDistance+1:distanceScaling
        x=int16(currentPosition+i*[sin(currentDirection+pi/4) cos(currentDirection+pi/4)]);
        if ~feasiblePoint(x,map), break; end
    end
    distanceFrontRightDiagonal=(i-robotHalfDiagonalDistance)/distanceScaling;
    
    % calculate angle deviation to goal
     slopeGoal=atan2(goal(1)-currentPosition(1),goal(2)-currentPosition(2));
     angleGoal=slopeGoal-currentDirection;
     while angleGoal>pi, angleGoal=angleGoal-2*pi; end % check to get the angle between -pi and pi
     while angleGoal<-pi, angleGoal=angleGoal+2*pi; end % check to get the angle between -pi and pi
     angleGoal=angleGoal/pi; % re-scaling the angle as per fuzzy modelling
    
     % calculate diatnce from goal
     distanceGoal=( sqrt(sum((currentPosition-goal).^2)) )/distanceScaling;
     if distanceGoal*distanceScaling<distanceThreshold, pathFound=true; end
     
     % calculate preferred turn. 
     % this indicates, if the front obstacle is far away, turn so as to more face the goal
     % if the front obstacle is close and a new front obstacle is encountered, turn using the side of the goal is preferred
     % if the front obstacle is close and the same obstacle as encountered in the previous step is found, same turn is made
     if (prevTurn==0 || prevTurn==1) && distanceFront<0.1 && (distanceFrontLeftDiagonal-prevDistanceLeftDiagonal)*distanceScaling<maxRobotSpeed, turn=1;
     elseif prevTurn==-1 && distanceFront<0.1 && (distanceFrontRightDiagonal-prevDistanceRightDiagonal)*distanceScaling<maxRobotSpeed, turn=-1;
     else turn=(angleGoal>=0)*1+(angleGoal<0)*(-1);prevTurn=turn;
     end
     prevDistanceLeftDiagonal=distanceFrontRightDiagonal;
     prevDistanceRightDiagonal=distanceFrontLeftDiagonal;
     
     % pass all computed inputs to a fuzzy inference system
     computedSteer=evalfis([distanceFront distanceFrontLeftDiagonal distanceFrontRightDiagonal angleGoal turn distanceGoal],fuz);
     currentDirection=currentDirection+computedSteer*directionScaling;
     
     % speed is set based on the front and diagonal distance so as not to make the robot collide, but make it slow and even stop before possible collission
     % distances here include additional safety distance of S
     distanceFrontSafety=max([distanceFront*distanceScaling-S 0]); 
     distanceFrontLeftDiagonalSafety=max([distanceFrontLeftDiagonal*distanceScaling-S 0]);
     distanceFrontRightDiagonalSafety=max([distanceFrontRightDiagonal*distanceScaling-S 0]);
     
     % maximum speeds admissible as per the above safety distance
     maxSpeed1=min([sqrt(2*maxAcceleration*distanceFrontSafety) maxRobotSpeed]);
     maxSpeed2=min([sqrt(maxAcceleration*distanceFrontLeftDiagonalSafety) maxRobotSpeed]);
     maxSpeed3=min([sqrt(maxAcceleration*distanceFrontRightDiagonalSafety) maxRobotSpeed]);
     maxSpeed=min([maxSpeed1 maxSpeed2 maxSpeed3]);
     
     % setting the speed based on vehicle acceleration and speed limits. the vehicle cannot move backwards.
     preferredSpeed=min([robotSpeed+maxAcceleration maxSpeed]);
     robotSpeed=max([robotSpeed-maxAcceleration preferredSpeed]);
     robotSpeed=min([robotSpeed maxRobotSpeed]);
     robotSpeed=max([robotSpeed 0]);
     
     if robotSpeed==0, error('robot had to stop to avoid collission'); end
     
     % calculating new position based on steer and speed
     newPosition=currentPosition+robotSpeed*[sin(currentDirection) cos(currentDirection)];
     pathCost=pathCost+distanceCost(newPosition,currentPosition);
     currentPosition=newPosition;
     if ~feasiblePoint(int16(currentPosition),map), error('collission recorded'); end
     
     % plotting robot
     if ~plotRobot(currentPosition,currentDirection,map,robotHalfDiagonalDistance)
        error('collission recorded');
     end
     M(t)=getframe;t=t+1;
end
fprintf('processing time=%d \nPath Length=%d \n\n', toc,pathCost); 