clear all;
close all;

RowGap = 19;
ColGap = 2.7;

ParkLotRowNum = 5;
ParkLotColNum = 20;

% map = [1 1 1 0 0 0 1 1 1 1 0 0 1];%worst case case 1 optimal
% map = [1 0 1 0 0 1 1 1 0 1 0 1 1];%
% map = [1 1 1 0 1 0 0 1 0 1 0 1 1];%

% map = [1 1 1 0 1 0 0 1 0 1 1 1 1 1 1 0 1 1 1 1 1];%

map = [1 1 1 1 1 1 1 1 0 0 1 0 1 0 1 1 0 0 0 0 1 0 1 0 1 1 0 1 0 0 0 0 1 0 1 1 0 1 1 1 1 0 0 1 0 1 0 1 1 0 0 0 0 1 0 1 0 1 1 0 1 0 0 0 0 1 1 1 1 1 0 0 1 0 1 0 1 1 0 0 0 0 1 0 1 0 1 1 0 1 0 0 1 0 0 1 1 1 0 1 1];%


AvailNum = 48;%total avaiable parking spots
UnAvailNum = ParkLotRowNum*ParkLotColNum - AvailNum;


RowSearchSequence = unique_perms([1:ParkLotRowNum]);

Coordinate = coordinate_generate(ParkLotRowNum, ParkLotColNum, [1,1], RowGap, ColGap);
Coordinate(ParkLotRowNum*ParkLotColNum+1,:) = Coordinate(length(Coordinate(:,1)),:) + [0 1];

for i = 1:length(RowSearchSequence)
    ExploreSequence(i).RowSequence = RowSearchSequence(i,:);
end


for i = 1:length(RowSearchSequence)
    ExploreSequence(i).RowSequence = RowSearchSequence(i,:);
    route = RouteGenerate(ParkLotRowNum, ParkLotColNum, ExploreSequence(i).RowSequence);
    ExploreSequence(i).IndexSequence = route.TraversalSequence;
end
rowSequenceTotal = [];

for i = 1:length(RowSearchSequence)
    ExploreSequence(i).map(1) = 1;
    ExploreSequence(i).Active = 1;
    ExploreSequence(i).NodeExplored = [ExploreSequence(i).IndexSequence(1)];
    ExploreSequence(i).Node2Explore = ExploreSequence(i).IndexSequence(2:end);
    ExploreSequence(i).obsearved = 0;
    ExploreSequence(i).ActionHistory = [];
    ExploreSequence(i).CurrentPosition = 1;
    ExploreSequence(i).flag = 1;
    ExploreSequence(i).unavailablePosition = [];
    ExploreSequence(i).obsearvedMap = [];
    ExploreSequence(i).AvailNum = AvailNum;
    ExploreSequence(i).UnAvailNum = UnAvailNum;
    
%     ExploreSequence(i).Map4Planning = [];
    
        
%     ExploreSequence(i).Map4Planning = MapGenerate(ExploreSequence(i).IndexSequence(2:end),AvailNum,UnAvailNum);
    for j = 1:length(ExploreSequence(i).IndexSequence)
        ExploreSequence(i).map(j) = map(ExploreSequence(i).IndexSequence(j));
    end
    rowSequenceTotal = [rowSequenceTotal;ExploreSequence(i).RowSequence];
end

velNum = 1;

DriveCost = 1;
WalkCost = 10;

for i = 1:100
%     ActiveIndex = [];
%     Cost2Compare = [];
    mmm = 0;
    for iii = 1:length(RowSearchSequence)
        if ExploreSequence(iii).Active == 0
            mmm = mmm + 1;
        end
    end
    if mmm == length(RowSearchSequence)
        break;
    end
    
    for j = 1:length(RowSearchSequence)
        if ExploreSequence(j).Active == 1
            AvailNum = ExploreSequence(j).AvailNum;
            UnAvailNum = ExploreSequence(j).UnAvailNum;
            break;
        end
    end
    
    for j = 1:length(RowSearchSequence)
        if ExploreSequence(j).Active == 1
            ExploreSequence(j).AllPossibleMap = [];
        end
    end
    factorialNum = factorial(AvailNum + UnAvailNum)/factorial(UnAvailNum)/factorial(AvailNum);
    
    if factorialNum > 50
        for qqq = 1:50
            m = [zeros(1,AvailNum),ones(1,UnAvailNum)];
            newSortingOrder = randperm(length(m));
            mScrambled = m(newSortingOrder);
            
            for j = 1:length(RowSearchSequence)
                if ExploreSequence(j).Active == 1
                    mScrambled = [mScrambled,ExploreSequence(j).flag,ExploreSequence(j).obsearvedMap];
                    ExploreSequence(j).AllPossibleMap = [ExploreSequence(j).AllPossibleMap;mScrambled];
                end
            end
        end
        AllPossibleMapNum = 50;
    else
        for j = 1:length(RowSearchSequence)
            if ExploreSequence(j).Active == 1
                ExploreSequence(j).AllPossibleMap = unique_perms([zeros(1,AvailNum),ones(1,UnAvailNum)]);
                ExploreSequence(j).AllPossibleMap(:,end+1) = ExploreSequence(j).flag;
                obsearvedMapNum = length(ExploreSequence(j).obsearvedMap);
                if obsearvedMapNum > 0
                    for uuu = 1:obsearvedMapNum
                        ExploreSequence(j).AllPossibleMap(:,end+1) = ExploreSequence(j).obsearvedMap(uuu);
                    end    
                end
                AllPossibleMapNum = size(ExploreSequence(j).AllPossibleMap);
            end
        end
        
    end
    
    for j = 1:length(RowSearchSequence)
        if ExploreSequence(j).Active == 1
            ExploreSequence(j).Map4Planning = [];
        end
    end
    if AllPossibleMapNum(1) > 20 
            
        for ggg = 1:AllPossibleMapNum(1)
            
            if rand(1) <= 0.1
                
                for j = 1:length(RowSearchSequence)
                    MapInter = [];
                    for ttt = ExploreSequence(j).CurrentPosition+1:length(ExploreSequence(j).IndexSequence)
                        MapInter = [MapInter ExploreSequence(j).AllPossibleMap(ggg,ExploreSequence(j).IndexSequence(ttt))];
                    end
                    ExploreSequence(j).Map4Planning = [ExploreSequence(j).Map4Planning;[ExploreSequence(j).obsearvedMap MapInter]];
                end
                
            end
            
        end
    else 
        for ggg = 1:AllPossibleMapNum(1)
            for j = 1:length(RowSearchSequence)
                MapInter = [];
                for ttt = ExploreSequence(j).CurrentPosition+1:length(ExploreSequence(j).IndexSequence)
                    MapInter = [MapInter ExploreSequence(j).AllPossibleMap(ggg,ExploreSequence(j).IndexSequence(ttt))];
                end
                ExploreSequence(j).Map4Planning = [ExploreSequence(j).Map4Planning;[ExploreSequence(j).obsearvedMap MapInter]];
            end
        end
        
    end
    
    
    nextNode = [];
    
    for j = 1:length(RowSearchSequence)
        if ExploreSequence(j).Active == 1
            nextNodeIndex = ExploreSequence(j).IndexSequence((ExploreSequence(j).CurrentPosition) + 1);
            if isempty(nextNode) == 1 
                nextNode(1).node = nextNodeIndex;
                nextNode(1).rowSequence = ExploreSequence(j).RowSequence;
            else
                nextNodeIndexList = [];
                for iii = 1:length(nextNode)
                    nextNodeIndexList = [nextNodeIndexList,nextNode(iii).node];
                end
                if ismember(nextNodeIndex,nextNodeIndexList)
                    [a,b] = ismember(nextNodeIndex,nextNodeIndexList);
                    nextNode(b).rowSequence = [nextNode(b).rowSequence;ExploreSequence(j).RowSequence];
                else
                    nextNode(length(nextNode)+1).node = nextNodeIndex;
                    nextNode(length(nextNode)).rowSequence = ExploreSequence(j).RowSequence;
                end
            end
        end
    end
    
    for ik = 1:length(nextNode)

        compareMatrix = [];
        nextNodeRowSequenceNum = size(nextNode(ik).rowSequence);
        for k = 1:nextNodeRowSequenceNum(1)
            [a,b] = ismember([nextNode(ik).rowSequence(k,:)],rowSequenceTotal,'rows');
            nextNode(ik).rowSequenceIndex(k) = b;
        end
        
        for k = 1:nextNodeRowSequenceNum(1)
            j = nextNode(ik).rowSequenceIndex(k);
            ExploreSequence(j).TotalParkSpotLeft = length(ExploreSequence(j).Node2Explore);

            Map4Planning = ExploreSequence(j).Map4Planning;
            Policy2Evaluate = PolicyGeneration(ExploreSequence(j).CurrentPosition,DriveCost,WalkCost,Coordinate,ExploreSequence(j).IndexSequence);%fix this
            Map4PlanningNum = size(Map4Planning);
            Policy2EvaluateNum = size(Policy2Evaluate);
            Cost2Go = zeros(Map4PlanningNum(1),1);
                
            for kk = 1:Map4PlanningNum(1)
                
                [MinCost,OptimalAction,index] = BestAction(Map4Planning(kk,:),Policy2Evaluate,ExploreSequence(j).CurrentPosition,ExploreSequence(j).flag);

                Cost2Go(kk,1) = MinCost;
            end
            compareMatrix = [compareMatrix;Cost2Go'];
                
%            
        end
%         
        compareMatrixNum = size(compareMatrix);
        cost2compare = [];
        for ppp = 1:compareMatrixNum(2)
            cost2compare = [cost2compare min(compareMatrix(:,ppp))];
        end
        
        nextNode(ik).maxValue = max(cost2compare);
        
    end
    maxOverallList = [];
    for ik = 1:length(nextNode)
        maxOverallList = [maxOverallList;nextNode(ik).maxValue];
    end
    minOverall = min(maxOverallList);
    checkNum = [];
    for ik = 1:length(nextNode)
        if nextNode(ik).maxValue == minOverall
            checkNum = [checkNum,ik];
        end
        if nextNode(ik).maxValue ~= minOverall
            for zz = 1:length(nextNode(ik).rowSequenceIndex)
                ExploreSequence(nextNode(ik).rowSequenceIndex(zz)).Active = 0;
            end
        end
    end
    if length(checkNum) > 1
        randpick = randsample(checkNum, length(checkNum)-1);
        for zzz = 1:length(randpick)
            for yyy = 1:length(nextNode(randpick(zzz)).rowSequenceIndex)
                ExploreSequence(nextNode(randpick(zzz)).rowSequenceIndex(yyy)).Active = 0;
            end
        end
    end
    
    for jj = 1:length(RowSearchSequence)
        
        if ExploreSequence(jj).Active == 1
            CurrentIndex = ExploreSequence(jj).IndexSequence(ExploreSequence(jj).CurrentPosition);
            CurrentCost = WalkCost*((Coordinate(CurrentIndex,1) - 1)^2 + (Coordinate(CurrentIndex,2) - 1)^2)^0.5;
            if ExploreSequence(jj).flag == 1
                Action = -1;

            elseif CurrentCost > minOverall
                Action = -1;
            else
                Action = 0;
            end
            
            ExploreSequence(jj).ActionHistory = [ExploreSequence(jj).ActionHistory Action];
            if Action == 0
                ExploreSequence(jj).Active = 0;
%                 break;
            else
                ExploreSequence(jj).CurrentPosition = ExploreSequence(jj).CurrentPosition + 1;
                ExploreSequence(jj).NodeExplored = [ExploreSequence(jj).NodeExplored ExploreSequence(jj).IndexSequence(ExploreSequence(jj).CurrentPosition)];
                ExploreSequence(jj).Node2Explore(1) = [];
            end
            if ExploreSequence(jj).map(1, ExploreSequence(jj).CurrentPosition) == 1 && ismember(ExploreSequence(jj).IndexSequence(ExploreSequence(jj).CurrentPosition),ExploreSequence(jj).NodeExplored(1:end-1)) == 0
                ExploreSequence(jj).obsearved = ExploreSequence(jj).obsearved + 1;
                ExploreSequence(jj).flag = 1;
                
                ExploreSequence(jj).UnAvailNum = ExploreSequence(jj).UnAvailNum - 1;
           
            elseif ExploreSequence(jj).map(1, ExploreSequence(jj).CurrentPosition) == 1 && ismember(ExploreSequence(jj).IndexSequence(ExploreSequence(jj).CurrentPosition),ExploreSequence(jj).NodeExplored(1:end-1)) == 1
                ExploreSequence(jj).flag = 1;
            elseif ExploreSequence(jj).map(1, ExploreSequence(jj).CurrentPosition) == 0 && ismember(ExploreSequence(jj).IndexSequence(ExploreSequence(jj).CurrentPosition),ExploreSequence(jj).NodeExplored(1:end-1)) == 0
                ExploreSequence(jj).flag = 0;
                ExploreSequence(jj).AvailNum = ExploreSequence(jj).AvailNum - 1;
            elseif ExploreSequence(jj).map(1, ExploreSequence(jj).CurrentPosition) == 0 && ismember(ExploreSequence(jj).IndexSequence(ExploreSequence(jj).CurrentPosition),ExploreSequence(jj).NodeExplored(1:end-1)) == 1
                ExploreSequence(jj).flag = 0;
            end
            ExploreSequence(jj).obsearvedMap = [ExploreSequence(jj).obsearvedMap ExploreSequence(jj).flag];
        end
        
    end
end
        
    
figure(4)
% axis equal;
title({'Parking Spot Search with','Sampling Game Theoretic Planning Algorithm'},'fontsize',14);
hold on;
set(gca,'FontSize',14);
axis equal;




% xticks([1 2 3 4 5]);
% yticks([1 2 3]);
% for i = 1:length(map)
%     if map(i) == 0
%         plot(Coordinate(i,2),(ParkLotRowNum-1)*RowGap + 2 - Coordinate(i,1),'o','MarkerSize',20,'MarkerFaceColor','g','MarkerEdgeColor','g');
%         hold on;
%     else
%         plot(Coordinate(i,2),(ParkLotRowNum-1)*RowGap + 2 - Coordinate(i,1),'x','MarkerSize',20,'MarkerEdgeColor','r');
%         hold on;
%     end
% end
% plot(Coordinate(ParkLotColNum*ParkLotRowNum+1,2),(ParkLotRowNum-1)*RowGap + 2 - Coordinate(ParkLotColNum*ParkLotRowNum+1,1),'*','MarkerSize',20,'MarkerEdgeColor','b');
% hold on;
vehicleDims.RearOverhang = 1;
vehicleDims.FrontOverhang = 1;
vehicleDims.Wheelbase = 2.5;
vehicleDims.Width = 1.75;
vehicleDims.Length = 4.5;
steer = 0;
vehiclePose = [Coordinate(ParkLotColNum*ParkLotRowNum+1,2),(ParkLotRowNum-1)*RowGap + 2 - Coordinate(ParkLotColNum*ParkLotRowNum+1,1),180];
[E1,E2] = EgoVehPlot(vehicleDims,vehiclePose,steer);
plot(Coordinate(1,2),(ParkLotRowNum-1)*RowGap + 2 - Coordinate(1,1),'-p','MarkerSize',15,'MarkerEdgeColor','b');
hold on;

for i = 1:ParkLotRowNum
    if mod(i,2) ~= 0
        line([Coordinate(2+ParkLotColNum*(i-1),2)-0.5*ColGap, Coordinate(ParkLotColNum*i-1,2)+0.5*ColGap],[Coordinate(2+ParkLotColNum*(i-1),1)-(1+RowGap/2/ColGap-1)*ColGap, Coordinate(1+ParkLotColNum*(i-1),1)-(1+RowGap/2/ColGap-1)*ColGap]);
        hold on;
        line([Coordinate(2+ParkLotColNum*(i-1),2)-0.5*ColGap, Coordinate(ParkLotColNum*i-1,2)+0.5*ColGap],[Coordinate(2+ParkLotColNum*(i-1),1)+(1+RowGap/2/ColGap-1)*ColGap, Coordinate(1+ParkLotColNum*(i-1),1)+(1+RowGap/2/ColGap-1)*ColGap]);
        hold on;
    else
        line([Coordinate(2+ParkLotColNum*(i-1),2)+0.5*ColGap, Coordinate(ParkLotColNum*i-1,2)-0.5*ColGap],[Coordinate(2+ParkLotColNum*(i-1),1)-(1+RowGap/2/ColGap-1)*ColGap, Coordinate(1+ParkLotColNum*(i-1),1)-(1+RowGap/2/ColGap-1)*ColGap]);
        hold on;
        line([Coordinate(2+ParkLotColNum*(i-1),2)+0.5*ColGap, Coordinate(ParkLotColNum*i-1,2)-0.5*ColGap],[Coordinate(2+ParkLotColNum*(i-1),1)+(1+RowGap/2/ColGap-1)*ColGap, Coordinate(1+ParkLotColNum*(i-1),1)+(1+RowGap/2/ColGap-1)*ColGap]);
        hold on;
    end
end
for i = 1:ParkLotRowNum
    if mod(i,2) ~= 0
        for j = 1:ParkLotColNum-1
            line([Coordinate(2+ParkLotColNum*(i-1),2)-0.5*ColGap+(j-1)*ColGap,Coordinate(2+ParkLotColNum*(i-1),2)-0.5*ColGap+(j-1)*ColGap],[Coordinate(1+ParkLotColNum*(i-1),1)-(1+RowGap/2/ColGap-1)*ColGap,Coordinate(1+ParkLotColNum*(i-1),1)-(1+RowGap/2/ColGap-1)*ColGap+6/6.7*(RowGap/2/ColGap-1)*ColGap]);
            line([Coordinate(2+ParkLotColNum*(i-1),2)-0.5*ColGap+(j-1)*ColGap,Coordinate(2+ParkLotColNum*(i-1),2)-0.5*ColGap+(j-1)*ColGap],[Coordinate(1+ParkLotColNum*(i-1),1)+(1+RowGap/2/ColGap-1)*ColGap,Coordinate(1+ParkLotColNum*(i-1),1)+(1+RowGap/2/ColGap-1)*ColGap-6/6.7*(RowGap/2/ColGap-1)*ColGap]);
        end
    else
        for j = 1:ParkLotColNum-1
            line([Coordinate(2+ParkLotColNum*(i-1),2)+0.5*ColGap+(j-1)*ColGap-(ParkLotColNum-2)*ColGap,Coordinate(2+ParkLotColNum*(i-1),2)+0.5*ColGap+(j-1)*ColGap-(ParkLotColNum-2)*ColGap],[Coordinate(1+ParkLotColNum*(i-1),1)-(1+RowGap/2/ColGap-1)*ColGap,Coordinate(1+ParkLotColNum*(i-1),1)-(1+RowGap/2/ColGap-1)*ColGap+6/6.7*(RowGap/2/ColGap-1)*ColGap]);
            line([Coordinate(2+ParkLotColNum*(i-1),2)+0.5*ColGap+(j-1)*ColGap-(ParkLotColNum-2)*ColGap,Coordinate(2+ParkLotColNum*(i-1),2)+0.5*ColGap+(j-1)*ColGap-(ParkLotColNum-2)*ColGap],[Coordinate(1+ParkLotColNum*(i-1),1)+(1+RowGap/2/ColGap-1)*ColGap,Coordinate(1+ParkLotColNum*(i-1),1)+(1+RowGap/2/ColGap-1)*ColGap-6/6.7*(RowGap/2/ColGap-1)*ColGap]);
        end
    end
    
end


vehicleDims.RearOverhang = 1;
vehicleDims.FrontOverhang = 1;
vehicleDims.Wheelbase = 2.5;
vehicleDims.Width = 1.75;
vehicleDims.Length = 4.5;
steer = 0;

mapzeroindex = [];
for i = 1:length(map)
    if map(i) == 0
        mapzeroindex = [mapzeroindex i];
    end
end
for i = 1:ParkLotRowNum
    zeroindexmatrix(i).index = [];
end
for i = 1:length(mapzeroindex)
    j = ceil(mapzeroindex(i)/ParkLotColNum);
    zeroindexmatrix(j).index(end+1) = mapzeroindex(i);
end
UnavailCoordinate = [];
for i = 1:ParkLotRowNum
    if mod(i,2) ~= 0
        for j = 1:ParkLotColNum-2
            
            if ismember(j,mod(zeroindexmatrix(ParkLotRowNum+ 1 -i).index,ParkLotColNum)-1) == 0
                
                
                vehiclePose = [Coordinate(2+ParkLotColNum*(i-1),2)+(j-1)*ColGap,Coordinate(1+ParkLotColNum*(i-1),1)-(1+RowGap/2/ColGap-1)*ColGap+1.5,90];
                VehPlot(vehicleDims,vehiclePose,steer);
                vehiclePose = [Coordinate(2+ParkLotColNum*(i-1),2)+(j-1)*ColGap,Coordinate(1+ParkLotColNum*(i-1),1)+(1+RowGap/2/ColGap-1)*ColGap-1.5,-90];
                VehPlot(vehicleDims,vehiclePose,steer);
            else
                if rand(1) > 0.5
                    vehiclePose = [Coordinate(2+ParkLotColNum*(i-1),2)+(j-1)*ColGap,Coordinate(1+ParkLotColNum*(i-1),1)-(1+RowGap/2/ColGap-1)*ColGap+1.5,90];
                    VehPlot(vehicleDims,vehiclePose,steer);
                    UnavailCoordinate = [UnavailCoordinate;[Coordinate(2+ParkLotColNum*(i-1),2)+(j-1)*ColGap,Coordinate(1+ParkLotColNum*(i-1),1)-(1+RowGap/2/ColGap-1)*ColGap+1.5,Coordinate(1+ParkLotColNum*(i-1),1)]];
                else
                    vehiclePose = [Coordinate(2+ParkLotColNum*(i-1),2)+(j-1)*ColGap,Coordinate(1+ParkLotColNum*(i-1),1)+(1+RowGap/2/ColGap-1)*ColGap-1.5,-90];
                    VehPlot(vehicleDims,vehiclePose,steer);
                    UnavailCoordinate = [UnavailCoordinate;[Coordinate(2+ParkLotColNum*(i-1),2)+(j-1)*ColGap,Coordinate(1+ParkLotColNum*(i-1),1)+(1+RowGap/2/ColGap-1)*ColGap-1.5,Coordinate(1+ParkLotColNum*(i-1),1)]];
                end
            end
        end
    else
        for j = 1:ParkLotColNum-2
            
            if ismember(ParkLotColNum - 1 - j,mod(zeroindexmatrix(ParkLotRowNum+ 1-i).index,ParkLotColNum)-1) == 0
                
                vehiclePose = [Coordinate(2+ParkLotColNum*(i-1),2)+ColGap+(j-1)*ColGap-(ParkLotColNum-2)*ColGap,Coordinate(1+ParkLotColNum*(i-1),1)-(1+RowGap/2/ColGap-1)*ColGap+1.5,90];
                VehPlot(vehicleDims,vehiclePose,steer);
                vehiclePose = [Coordinate(2+ParkLotColNum*(i-1),2)+ColGap+(j-1)*ColGap-(ParkLotColNum-2)*ColGap,Coordinate(1+ParkLotColNum*(i-1),1)+(1+RowGap/2/ColGap-1)*ColGap-1.5,-90];
                VehPlot(vehicleDims,vehiclePose,steer);
            else
                if rand(1) > 0.5
                    vehiclePose = [Coordinate(2+ParkLotColNum*(i-1),2)+ColGap+(j-1)*ColGap-(ParkLotColNum-2)*ColGap,Coordinate(1+ParkLotColNum*(i-1),1)-(1+RowGap/2/ColGap-1)*ColGap+1.5,90];
                    VehPlot(vehicleDims,vehiclePose,steer);
                    UnavailCoordinate = [UnavailCoordinate;[Coordinate(2+ParkLotColNum*(i-1),2)+ColGap+(j-1)*ColGap-(ParkLotColNum-2)*ColGap,Coordinate(1+ParkLotColNum*(i-1),1)-(1+RowGap/2/ColGap-1)*ColGap+1.5,Coordinate(1+ParkLotColNum*(i-1),1)]];
                else
                    vehiclePose = [Coordinate(2+ParkLotColNum*(i-1),2)+ColGap+(j-1)*ColGap-(ParkLotColNum-2)*ColGap,Coordinate(1+ParkLotColNum*(i-1),1)+(1+RowGap/2/ColGap-1)*ColGap-1.5,-90];
                    VehPlot(vehicleDims,vehiclePose,steer);
                    UnavailCoordinate = [UnavailCoordinate;[Coordinate(2+ParkLotColNum*(i-1),2)+ColGap+(j-1)*ColGap-(ParkLotColNum-2)*ColGap,Coordinate(1+ParkLotColNum*(i-1),1)+(1+RowGap/2/ColGap-1)*ColGap-1.5,Coordinate(1+ParkLotColNum*(i-1),1)]];
                end
            end
        end
    end
end
    



pause(2);
for i = 1:length(ExploreSequence)
    if ismember(0,ExploreSequence(i).ActionHistory)
        break;
    end
end

for k = 1:length(ExploreSequence(i).NodeExplored)-1
    if k == 1 && length(ExploreSequence(i).NodeExplored) - 1 == 1
        pause(1);
        vehiclePose = [Coordinate(ExploreSequence(i).NodeExplored(k+1),2),(ParkLotRowNum-1)*RowGap + 2 - Coordinate(ExploreSequence(i).NodeExplored(k+1),1),180];
        delete(E1);
        delete(E2);
        [E1,E2] = EgoVehPlot(vehicleDims,vehiclePose,steer);
        line([Coordinate(ParkLotColNum*ParkLotRowNum+1,2),Coordinate(ExploreSequence(i).NodeExplored(k+1),2)],[(ParkLotRowNum-1)*RowGap + 2 - Coordinate(ParkLotColNum*ParkLotRowNum+1,1),(ParkLotRowNum-1)*RowGap + 2 - Coordinate(ExploreSequence(i).NodeExplored(k+1),1)],'LineWidth',3);
    elseif k == 1 && length(ExploreSequence(i).NodeExplored) > 2 
        vehiclePose = [Coordinate(ExploreSequence(i).NodeExplored(k+1),2),(ParkLotRowNum-1)*RowGap + 2 - Coordinate(ExploreSequence(i).NodeExplored(k+1),1),180];
        delete(E1);
        delete(E2);
        [E1,E2] = EgoVehPlot(vehicleDims,vehiclePose,steer);
        line([Coordinate(ParkLotColNum*ParkLotRowNum+1,2),Coordinate(ExploreSequence(i).NodeExplored(k+1),2)],[(ParkLotRowNum-1)*RowGap + 2 - Coordinate(ParkLotColNum*ParkLotRowNum+1,1),(ParkLotRowNum-1)*RowGap + 2 - Coordinate(ExploreSequence(i).NodeExplored(k+1),1)],'LineWidth',3);
    elseif k ~= length(ExploreSequence(i).NodeExplored)-1
        if (ParkLotRowNum-1)*RowGap + 2 - Coordinate(ExploreSequence(i).NodeExplored(k),1)==(ParkLotRowNum-1)*RowGap + 2 - Coordinate(ExploreSequence(i).NodeExplored(k+1),1) && Coordinate(ExploreSequence(i).NodeExplored(k),2) < Coordinate(ExploreSequence(i).NodeExplored(k+1),2)
            oritation = 180;
        elseif (ParkLotRowNum-1)*RowGap + 2 - Coordinate(ExploreSequence(i).NodeExplored(k),1)==(ParkLotRowNum-1)*RowGap + 2 - Coordinate(ExploreSequence(i).NodeExplored(k+1),1) && Coordinate(ExploreSequence(i).NodeExplored(k),2) > Coordinate(ExploreSequence(i).NodeExplored(k+1),2)
            oritation = -180;
        elseif  Coordinate(ExploreSequence(i).NodeExplored(k),2) == Coordinate(ExploreSequence(i).NodeExplored(k+1),2) && (ParkLotRowNum-1)*RowGap + 2 - Coordinate(ExploreSequence(i).NodeExplored(k),1) < (ParkLotRowNum-1)*RowGap + 2 - Coordinate(ExploreSequence(i).NodeExplored(k+1),1)
            oritation = 90;
        elseif  Coordinate(ExploreSequence(i).NodeExplored(k),2) == Coordinate(ExploreSequence(i).NodeExplored(k+1),2) && (ParkLotRowNum-1)*RowGap + 2 - Coordinate(ExploreSequence(i).NodeExplored(k),1) > (ParkLotRowNum-1)*RowGap + 2 - Coordinate(ExploreSequence(i).NodeExplored(k+1),1)
            oritation = -90;
        end
            
        vehiclePose = [Coordinate(ExploreSequence(i).NodeExplored(k+1),2),(ParkLotRowNum-1)*RowGap + 2 - Coordinate(ExploreSequence(i).NodeExplored(k+1),1),oritation];
        delete(E1);
        delete(E2);
        [E1,E2] = EgoVehPlot(vehicleDims,vehiclePose,steer);
        line([Coordinate(ExploreSequence(i).NodeExplored(k),2),Coordinate(ExploreSequence(i).NodeExplored(k+1),2)],[(ParkLotRowNum-1)*RowGap + 2 - Coordinate(ExploreSequence(i).NodeExplored(k),1),(ParkLotRowNum-1)*RowGap + 2 - Coordinate(ExploreSequence(i).NodeExplored(k+1),1)],'LineWidth',3);
    else
        if (ParkLotRowNum-1)*RowGap + 2 - Coordinate(ExploreSequence(i).NodeExplored(k),1)==(ParkLotRowNum-1)*RowGap + 2 - Coordinate(ExploreSequence(i).NodeExplored(k+1),1) && Coordinate(ExploreSequence(i).NodeExplored(k),2) < Coordinate(ExploreSequence(i).NodeExplored(k+1),2)
            oritation = 180;
        elseif (ParkLotRowNum-1)*RowGap + 2 - Coordinate(ExploreSequence(i).NodeExplored(k),1)==(ParkLotRowNum-1)*RowGap + 2 - Coordinate(ExploreSequence(i).NodeExplored(k+1),1) && Coordinate(ExploreSequence(i).NodeExplored(k),2) > Coordinate(ExploreSequence(i).NodeExplored(k+1),2)
            oritation = -180;
        elseif  Coordinate(ExploreSequence(i).NodeExplored(k),2) == Coordinate(ExploreSequence(i).NodeExplored(k+1),2) && (ParkLotRowNum-1)*RowGap + 2 - Coordinate(ExploreSequence(i).NodeExplored(k),1) < (ParkLotRowNum-1)*RowGap + 2 - Coordinate(ExploreSequence(i).NodeExplored(k+1),1)
            oritation = 90;
        elseif  Coordinate(ExploreSequence(i).NodeExplored(k),2) == Coordinate(ExploreSequence(i).NodeExplored(k+1),2) && (ParkLotRowNum-1)*RowGap + 2 - Coordinate(ExploreSequence(i).NodeExplored(k),1) > (ParkLotRowNum-1)*RowGap + 2 - Coordinate(ExploreSequence(i).NodeExplored(k+1),1)
            oritation = -90;
        end
            
        vehiclePose = [Coordinate(ExploreSequence(i).NodeExplored(k+1),2),(ParkLotRowNum-1)*RowGap + 2 - Coordinate(ExploreSequence(i).NodeExplored(k+1),1),oritation];
        delete(E1);
        delete(E2);
        [E1,E2] = EgoVehPlot(vehicleDims,vehiclePose,steer);
        line([Coordinate(ExploreSequence(i).NodeExplored(k),2),Coordinate(ExploreSequence(i).NodeExplored(k+1),2)],[(ParkLotRowNum-1)*RowGap + 2 - Coordinate(ExploreSequence(i).NodeExplored(k),1),(ParkLotRowNum-1)*RowGap + 2 - Coordinate(ExploreSequence(i).NodeExplored(k+1),1)],'LineWidth',3);
    end
    pause(1);
end

UnavailCoordinateNum = size(UnavailCoordinate);

for j = 1:UnavailCoordinateNum(1)
    if find([UnavailCoordinate(j,1),UnavailCoordinate(j,3)] == [Coordinate(ExploreSequence(i).NodeExplored(k+1),2),(ParkLotRowNum-1)*RowGap + 2 - Coordinate(ExploreSequence(i).NodeExplored(k+1),1)])
        break;
    end
end

if (ParkLotRowNum-1)*RowGap + 2 - Coordinate(ExploreSequence(i).NodeExplored(k+1),1) -(1+RowGap/2/ColGap-1)*ColGap+1.5 ~= UnavailCoordinate(j,2)
    vehiclePose = [Coordinate(ExploreSequence(i).NodeExplored(k+1),2),(ParkLotRowNum-1)*RowGap + 2 - Coordinate(ExploreSequence(i).NodeExplored(k+1),1)+(1+RowGap/2/ColGap-1)*ColGap-2.5,90];
    delete(E1);
    delete(E2);
    EgoVehPlot(vehicleDims,vehiclePose,steer);
    line([Coordinate(ExploreSequence(i).NodeExplored(k+1),2),Coordinate(ExploreSequence(i).NodeExplored(k+1),2)],[(ParkLotRowNum-1)*RowGap + 2 - Coordinate(ExploreSequence(i).NodeExplored(k+1),1),(ParkLotRowNum-1)*RowGap + 2 - Coordinate(ExploreSequence(i).NodeExplored(k+1),1)+(1+RowGap/2/ColGap-1)*ColGap-2.5],'LineWidth',3);
else
    vehiclePose = [Coordinate(ExploreSequence(i).NodeExplored(k+1),2),(ParkLotRowNum-1)*RowGap + 2 - Coordinate(ExploreSequence(i).NodeExplored(k+1),1)-(1+RowGap/2/ColGap-1)*ColGap+2.5,270];
    delete(E1);
    delete(E2);
    EgoVehPlot(vehicleDims,vehiclePose,steer);
    line([Coordinate(ExploreSequence(i).NodeExplored(k+1),2),Coordinate(ExploreSequence(i).NodeExplored(k+1),2)],[(ParkLotRowNum-1)*RowGap + 2 - Coordinate(ExploreSequence(i).NodeExplored(k+1),1),(ParkLotRowNum-1)*RowGap + 2 - Coordinate(ExploreSequence(i).NodeExplored(k+1),1)-(1+RowGap/2/ColGap-1)*ColGap+2.5],'LineWidth',3);
end



