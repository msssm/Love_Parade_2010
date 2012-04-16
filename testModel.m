function [avgPplPerSq,pplPerSq]=testModel(runs)

%global variables
arenaSizex = 100;   %size of arena in x direction
arenaSizey = 100;   %size of arena in y direction
maxSize = max(arenaSizex,arenaSizey);   %maximum side length
maxPplPerSq = 8;    %maximum number of people allowed per square meter
pplPerSq = zeros(maxPplPerSq,runs); %A history of how many squares had n people
exitsPerRound = 3;  %number of people allowed to exit the grounds per iteration
meanWillFactor = 1;     %a persons average will factor
lineOfSight = 0;    %BOOL

%create figure
figure
hold off

[arena,exit,endZone,goal,titleText] = makeArenaNoExit(arenaSizex,arenaSizey);

aviobj = avifile('mymovie.avi','fps',25);
set(gcf,'position',[0 0 1200 500]);

%initialize List outside loop, this creates a slight speed improvement
initializeOutside = 0;
if initializeOutside == 1
    [~, idxSortedList] = sort(rand(1,arenaSizex*arenaSizey));   %generate a random list from 1 to arenaSizex*arenaSizey
end

%RUN SIMULATION
for runNumber = 1:runs
    
    %add people to the arena every iteration
    if ((mod(runNumber,100) <= 50)&& runNumber <1000)
        for xadd = 85:99
            arena(xadd,2) = 5; %add people on the left
            arena(xadd,arenaSizex-1) = 5; %... right
        end
    end
    
    if (runNumber == 1000) % change arena at 1000 steps
        arena(100,49:51) = 0.001i;  
        arena(66,39) = 0;
        arena(66,61) = 0;
        exit = [49 51 99 99;39 39 66 66;61 61 66 66]; %[xStart,xEnd,yStart,yEnd]
        endZone = [50 50 100 100;39 39 66 66;61 61 66 66];
        goal = [50 100;39 66;61 66];
        meanWillFactor = 8; %panic breaks out and people go crazy
        lineOfSight = 1;
    end
    
    %statistics
    for k = 1:maxPplPerSq
        pplPerSq(k,runNumber) = sum(sum(arena==k));
    end
    pplPerSq(maxPplPerSq,runNumber) = sum(sum(arena >= maxPplPerSq));
    
    %display figure
    clf     % Clear figure
    subplot(1,2,1);
    colormap([[0,linspace(32/255,221/255,8),0.85];[0,linspace(14/255,14/255,8),0.85];[0,linspace(246/255,246/255,8),0.85]]');
    imagesc(abs(arena), [0 10])                   % Display grid
    axis image;
    colorbar
    title (titleText,'fontsize',15)
    text(-40,10,strcat('Step:',int2str(runNumber)),'fontsize',15)
    text(-40,20,strcat('People:',int2str(sum(sum(real(arena))))),'fontsize',15)
    h = subplot(1,2,2);
    hold off
    axis manual
    
    bar(h,pplPerSq(:,runNumber)/sum(pplPerSq(:,runNumber)));
    axis([0.5 8.5 0 0.7])
    title ('Percentage of square occupancies');
    xlabel('Square occupancy');
    ylabel('Percentage of squares with x occupancy');
    frame = getframe(gcf);
    if (mod(runNumber,2) == 0)        
        aviobj = addframe(aviobj,frame);
    end
    
    %initialize Random Sequence
    if initializeOutside == 0
        [~, idxSortedList] = sort(rand(1,arenaSizex*arenaSizey));   
    end
    %a random list of every square
    squaresList = [mod((idxSortedList-1),arenaSizex)+1; floor((idxSortedList-1)/arenaSizey)+1]; %Generate a set of 100*100 randomly distributed coordinates

    %start moving people
    for randomSquare = squaresList   %iterate over all coordinates
        x = randomSquare(1);    %extract the current x coordinate
        y = randomSquare(2);    %... y coordinate
        if arena(y,x) <= 0
            continue    %skip cell if no one is there
        end

        %line of sight - ie. can people see the goals
        goalsInLineOfSight = zeros(0);  
        
        if lineOfSight == 1     
            for goalNumber = 1:size(goal,1);    %generate a list of what goals individuals can see from their current location
                oneGoal = goal(goalNumber,:);
                inSight = 1;
                dir = [oneGoal(1)-x (oneGoal(2)-y)]';
                if norm(dir) == 0
                    continue
                end
                dir = dir/norm(dir);
                for k = 1:maxSize
                    if norm(oneGoal' - ([x y]'+floor([dir(1) dir(2)])'*k)) < 3
                        break
                     end
                    if ~(y+dir(2)*k < 1 || y+dir(2)*k > arenaSizey || x+dir(1)*k < 1 || x+dir(1)*k > arenaSizex) ...
                            && imag(arena(y+floor(dir(2)*k),x+floor(dir(1)*k))) ~= 0
                        inSight = 0;
                        break
                    end
                end
                if inSight == 1 %if no wall is blocking the line of sight to a goal, then add that goal to the list
                    goalsInLineOfSight = [goalsInLineOfSight; oneGoal];
                end
            end
        end
        
        if norm(goalsInLineOfSight) == 0    %if no goals can be seen, then assume a person can see all goals
            goalsToTry = goal;
        else
            goalsToTry = goalsInLineOfSight;
        end

        bestDesiredDirection = [Inf Inf];   
        for goalNumber = 1:size(goalsToTry,1)   %find the closest goal
            if (x >= endZone(goalNumber,1)) && (x <= endZone(goalNumber,2)) && (y >= endZone(goalNumber,3)) && (y <= endZone(goalNumber,4))     
                desiredDirection = [0;0];   %if we are at the end zone then we don't want to move from our current position
                break
            end
            currentDesiredDirection = [(goalsToTry(goalNumber,1)-x) -(goalsToTry(goalNumber,2)-y)]';   
            if norm(currentDesiredDirection) < norm(bestDesiredDirection)  %is this goal closer than the last goal
                bestDesiredDirection = currentDesiredDirection;
                desiredDirection = bestDesiredDirection/norm(bestDesiredDirection);
            end
        end         
         
        for exitNumber = 1:size(exit,1)     %allow people to exit
             if x >= exit(exitNumber,1) && x <= exit(exitNumber,2) && y >= exit(exitNumber,3) && y <= exit(exitNumber,4)     
                 arena(y,x) = arena(y,x) - min(exitsPerRound, arena(y,x));
                 if arena(y,x) <= 0
                      continue    %skip cell if everyone has exited
                 end
             end
        end
     
        for person = 1:arena(y,x)   %iterate over every person in a cell
            arena(y,x) = arena(y,x)-1;  %remove person from position
            chosenSquare = intSquare(desiredDirection);

            if imag(arena(y-chosenSquare(2),x+chosenSquare(1))) ~= 0    %will we hit a wall? In which case we need to find where go to get to our goal
                dirNintyDeg = [0 -1;1 0] * chosenSquare;    %rotate the chosen direction 90Deg, ie. parallel to wall
                for k = 1:maxSize
                    if ~(y-(chosenSquare(2)+dirNintyDeg(2)*k) < 1 || y-(chosenSquare(2)+dirNintyDeg(2)*k) > arenaSizey ...    %make sure we are not testing outside of arena
                            || x+(chosenSquare(1)+dirNintyDeg(1)*k) < 1 || x+(chosenSquare(1)+dirNintyDeg(1)*k) > arenaSizex) ...
                             && imag(arena(y-(chosenSquare(2)+dirNintyDeg(2)*k),x+(chosenSquare(1)+dirNintyDeg(1)*k))) == 0     %have we found a non-wall position?
                        desiredDirection = dirNintyDeg/norm(dirNintyDeg);   %rotate chosen direction 90Deg
                        break
                    end
                    if ~(y-(chosenSquare(2)-dirNintyDeg(2)*k) < 1 || y-(chosenSquare(2)-dirNintyDeg(2)*k) > arenaSizey ...
                            || x+(chosenSquare(1)-dirNintyDeg(1)*k) < 1 || x+(chosenSquare(1)-dirNintyDeg(1)*k) > arenaSizex) ...
                            && imag(arena(y-(chosenSquare(2)-dirNintyDeg(2)*k),x+(chosenSquare(1)-dirNintyDeg(1)*k))) == 0
                        desiredDirection = -dirNintyDeg/norm(dirNintyDeg);  %rotate chosen direction -90Deg
                        break
                    end
                    if k == maxSize
                        desiredDirection = [0;0];   %could not find a way to get to our goal
                    end
                end
            end

            chosenDirection = movementFactor(arena(y-1:y+1,x-1:x+1), desiredDirection,meanWillFactor);  %calculate where we want to go
            actualDir = intSquare(chosenDirection); %convert this to a Moore neighborhood
            
            if imag(arena(y-actualDir(2),x+actualDir(1))) == 0 && arena(y-actualDir(2),x+actualDir(1)) < maxPplPerSq    %if our desired cell is not wall and the maxPpl has not been reached
                arena(y-actualDir(2),x+actualDir(1)) = arena(y-actualDir(2),x+actualDir(1)) + 1; %then place person in position
            else
                arena(y,x) = arena(y,x) + 1;   %else put person back where he came from
            end
        end
    end    
end

aviobj = close(aviobj);

%average
avgPplPerSq = mean(pplPerSq,2);
end

function [pressure]=movementFactor(mobilitySquare, desiredDirection,meanWillFactor)  %calculate a persons ability to move in a given direction
%variables
wallPressure =2; %how much additional force do walls apply, resp. people are unlikely to stay closer to walls
wallVariation = 0.5;  %vary the amount of "pressure" that walls applay
cornerFactor = 0.707;   %1/sqrt(2), People at a diagonal to you exert a smaller force
standardDevWillFactor = 1;  %how much does a persons will factor vary
claustrophobiaFactor = 1/5;     %how much is your movement inhibited by the presense of others

%Newtons third law
for j = 1:3
    for k = 1:3
        if imag(mobilitySquare(j,k))            
            mobilitySquare(j,k) = mobilitySquare(4-j,4-k)+wallPressure+wallVariation*abs(randn(1));  
        end
    end
end    


peopleYplus = sum(sum(real(mobilitySquare).*[cornerFactor 1 cornerFactor;0 0 0;0 0 0])); %number of people in positive y  direction
peopleYminus = sum(sum(real(mobilitySquare).*[0 0 0;0 0 0;cornerFactor 1 cornerFactor])); %... negative y ..
peopleXplus = sum(sum(real(mobilitySquare).*[cornerFactor 1 cornerFactor;0 0 0;0 0 0]'));   %... positive x ..
peopleXminus = sum(sum(real(mobilitySquare).*[0 0 0;0 0 0;cornerFactor 1 cornerFactor]'));  %... negative x ..
willFactor = standardDevWillFactor*(meanWillFactor+randn(1))*[1/(1+claustrophobiaFactor*min([peopleXplus,peopleXminus])); ...
    1/(1+claustrophobiaFactor*min([peopleYplus,peopleYminus]))]; %calculate a persons will and ability to move to a given cell 
pressure =  (diag([cornerFactor 1 cornerFactor;-1 0 1]*(real(mobilitySquare).^2)*[1 cornerFactor;0 1;-1 cornerFactor]) ...
    + willFactor.*desiredDirection);    %vector sum of pressure plus personal will
end

function [square]=intSquare(dir) %convert from a point in 2D cartesian space to a first Moore neighborhood
if abs(dir) < 3/2
    square = round(dir);
else 
    angl = angle([1 1i]*dir)-pi/16;
    if angl < 0
         angl = angl + 2*pi;
    end
    correspondanceMatrix = [1 1;0 1;-1 1;-1 0;-1 -1;0 -1;1 -1;1 0]';
    square = correspondanceMatrix(:,floor(angl*8/(2*pi))+1);
end
end

function[arena,exit,endZone,goal,titleText]=makeArenaNoExit(arenaSizex,arenaSizey)
%Initialize Arena Borders
arena = 10i*ones(arenaSizey,arenaSizex);

%initialize Arena
arena(85:arenaSizey-1,2:arenaSizex-1) = 0;
arena(2:arenaSizey-1,40:60) = 0;
arena(20,40:60)=10i;

%small arena
exit = [45,55,2,10]; %[xStart,xEnd,yStart,yEnd]
endZone = [45 55 2 10];
goal = [50 4]; %[goalx, goaly] goal must be within endZone

titleText = 'Simulation: no exit, entrance cyclical, 3 exits at 1000 steps';
end

function[arena,exit,endZone,goal,titleText]=makeArenaOneExit(arenaSizex,arenaSizey)
%Initialize Arena Borders
arena = 10i*ones(arenaSizey,arenaSizex);

%initialize Arena
arena(85:arenaSizey-1,2:arenaSizex-1) = 0;
arena(2:arenaSizey-1,40:60) = 0;

exit = [45,55,2,10;45 55 90 93]; %[xStart,xEnd,yStart,yEnd]
endZone = [45 55 2 10];
goal = [50 4]; %[goalx, goaly] goal must be within endZone

titleText = 'Simulation of arena with one main exit, entrance cyclical';
end

function[arena,exit,endZone,goal,titleText]=makeArenaOneExitDawdle(arenaSizex,arenaSizey)
%Initialize Arena Borders
arena = 10i*ones(arenaSizey,arenaSizex);

%initialize Arena
arena(85:arenaSizey-1,2:arenaSizex-1) = 0;
arena(2:arenaSizey-1,40:60) = 0;

exit = [45,55,2,3]; %[xStart,xEnd,yStart,yEnd]
endZone = [45 55 4 10];
goal = [50 4]; %[goalx, goaly] goal must be within endZone

titleText = 'Simulation of arena with one main exit, entrance cyclical';
end

function[arena,exit,endZone,goal]=makeArenaLarge(arenaSizex,arenaSizey)
%Initialize Arena Borders
arena = 10i*ones(arenaSizey,arenaSizex);

%initialize Arena
arena(85:arenaSizey-1,2:arenaSizex-1) = 0;
arena(2:arenaSizey-1,40:60) = 0;
arena(20,40:60)=10i;

exit = [45,55,2,10]; %[xStart,xEnd,yStart,yEnd]
endZone = [45 55 2 10];
goal = [50 4]; %[goalx, goaly] goal must be within endZone
end