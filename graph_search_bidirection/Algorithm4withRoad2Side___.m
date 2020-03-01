clear all;
close all;

RowGap = 19;
ColGap = 2.7;

ParkLotRowNum = 3;
ParkLotColNum = 16;



map = [1 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0 1 1];
% map = [1 1 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0 1 1 1 1 1 1 1 1 1];

AvailNum_actual = 3;

AvailNum_list = PossibleNumGenerate(AvailNum_actual);

AllPossibleMap = [];

for i = 1:length(AvailNum_list)
    AvailNum = AvailNum_list(i);%total avaiable parking spots
    UnAvailNum = ParkLotRowNum*ParkLotColNum - AvailNum - 2*ParkLotRowNum;
    
    AllPossibleMap_inter = unique_perms([zeros(1,AvailNum),ones(1,UnAvailNum)]);
    AllPossibleMap = [AllPossibleMap;AllPossibleMap_inter];   

end
AllPossibleMap(:,end+1) = 1;
AllPossibleMapNum = size(AllPossibleMap);

RowSearchSequence = unique_perms([1:ParkLotRowNum]);

Coordinate = coordinate_generate(ParkLotRowNum, ParkLotColNum, [1,1], RowGap, ColGap);
Coordinate(ParkLotRowNum*ParkLotColNum+1,:) = Coordinate(length(Coordinate(:,1)),:) + [0 ColGap];

RoadIndex = zeros(1,2*ParkLotRowNum);
for i = 1:2*ParkLotRowNum
    if mod(i,2) ~= 0
        RoadIndex(i) = 1 + ParkLotColNum*(i-1)/2;
    else
        RoadIndex(i) = ParkLotColNum*i/2;
    end
end
AllPossibleMapModified = ones(AllPossibleMapNum(1),AllPossibleMapNum(2) + 2*ParkLotRowNum);
for i = 1:ParkLotRowNum
    AllPossibleMapModified(:,RoadIndex(2*i-1)+1:RoadIndex(2*i)-1) = AllPossibleMap(:,(i-1)*(ParkLotColNum-2)+1:i*(ParkLotColNum-2));
end
AllPossibleMapModifiedNum = size(AllPossibleMapModified);
for i = 1:length(RowSearchSequence)
    ExploreSequence(i).RowSequence = RowSearchSequence(i,:);
    route = RouteGenerate(ParkLotRowNum, ParkLotColNum, ExploreSequence(i).RowSequence);
    ExploreSequence(i).IndexSequence = route.TraversalSequence;
end
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
    
    ExploreSequence(i).Map4Planning = [];
    for j = 1:AllPossibleMapModifiedNum(1)
        MapInter = [];
        for l = 2:length(ExploreSequence(i).IndexSequence)
            MapInter = [MapInter AllPossibleMapModified(j,ExploreSequence(i).IndexSequence(l))];
        end
        ExploreSequence(i).Map4Planning = [ExploreSequence(i).Map4Planning;MapInter];
    end
    
%     ExploreSequence(i).Map4Planning = MapGenerate(ExploreSequence(i).IndexSequence(2:end),AvailNum,UnAvailNum);
    for j = 1:length(ExploreSequence(i).IndexSequence)
        ExploreSequence(i).map(j) = map(ExploreSequence(i).IndexSequence(j));
    end
end

DriveCost = 1;
WalkCost = 1.2;


ActiveIndex = [];
Cost2Compare = [];
for i = 1:100
    ActiveIndex = [];
    Cost2Compare = [];
    mmm = 0;
    for iii = 1:length(RowSearchSequence)
        if ExploreSequence(iii).Active == 0
            mmm = mmm + 1;
        end
    end
    if mmm == length(RowSearchSequence)
        break;
    end
    compareMatrix = [];
    for j = 1:length(RowSearchSequence)
        
        if ExploreSequence(j).Active == 1
            
            ExploreSequence(j).TotalParkSpotLeft = length(ExploreSequence(j).Node2Explore);
            if isempty(ExploreSequence(j).obsearvedMap) == 1
                Map4Planning = ExploreSequence(j).Map4Planning;
            else
                Map2PlanningNum = size(ExploreSequence(j).Map4Planning);
                Map4Planning = [];
                for s = 1:Map2PlanningNum(1)
                    if ExploreSequence(j).obsearvedMap == ExploreSequence(j).Map4Planning(s,[1:ExploreSequence(j).CurrentPosition-1])
                        Map4Planning = [Map4Planning;ExploreSequence(j).Map4Planning(s,:)];
                    end
                end
            end
            Policy2Evaluate = PolicyGeneration(ExploreSequence(j).CurrentPosition,DriveCost,WalkCost,Coordinate,ExploreSequence(j).IndexSequence);%fix this
            Map4PlanningNum = size(Map4Planning);
            Policy2EvaluateNum = size(Policy2Evaluate);
            Cost2Go = zeros(Map4PlanningNum(1),1);
            for k = 1:Map4PlanningNum(1)
            
                [MinCost,OptimalAction,index] = BestAction(Map4Planning(k,:),Policy2Evaluate,ExploreSequence(j).CurrentPosition,ExploreSequence(j).flag);
                Cost2Go(k,1) = MinCost;
            end
            compareMatrix = [compareMatrix;Cost2Go'];
            ExploreSequence(j).Cost2Go = max(Cost2Go);
            
            ActiveIndex = [ActiveIndex j];
%             Cost2Compare = [Cost2Compare ExploreSequence(j).Cost2Go];
        end

    end
    
    compareMatrixNum = size(compareMatrix);
    ColMin = [];
    for mmm = 1:compareMatrixNum(2)
        ColMin = [ColMin min(compareMatrix(:,mmm))];
    end
    
    RowColMax = max(ColMin);
    
    BestActionIndex = [];
    
    for nnn = 1:length(ColMin)
        if ColMin(nnn) == RowColMax
            [minRouteValue,minRouteIndex] = min(compareMatrix(:,nnn));
            for rrr = 1:length(compareMatrix(:,nnn))
                if compareMatrix(rrr,nnn) == minRouteValue
                    BestActionIndex = [BestActionIndex ActiveIndex(rrr)];
                end
            end
        end
    end

    for ii = 1:length(RowSearchSequence)
        if i ~= 1 && (ExploreSequence(ii).Active == 1) && ismember(ii,BestActionIndex) == 0
            ExploreSequence(ii).Active = 0;
        end
    end
    for jj = 1:length(RowSearchSequence)
        
        if ExploreSequence(jj).Active == 1
            CurrentIndex = ExploreSequence(jj).IndexSequence(ExploreSequence(jj).CurrentPosition);
            CurrentCost = WalkCost*((Coordinate(CurrentIndex,1) - 1)^2 + (Coordinate(CurrentIndex,2) - 1)^2)^0.5;
%             CurrentCost = WalkCost*((Coordinate(CurrentIndex,1) - 1) + (Coordinate(CurrentIndex,2) - 1));
            if ExploreSequence(jj).flag == 1
                Action = -1;
                %             elseif CurrentCost > ExploreSequence(jj).Cost2Go
            elseif CurrentCost > RowColMax
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
                %             ExploreSequence(jj).unavailablePosition = ExploreSequence(jj).IndexSequence(ExploreSequence(jj).CurrentPosition);
            elseif ExploreSequence(jj).map(1, ExploreSequence(jj).CurrentPosition) == 1 && ismember(ExploreSequence(jj).IndexSequence(ExploreSequence(jj).CurrentPosition),ExploreSequence(jj).NodeExplored(1:end-1)) == 1
                ExploreSequence(jj).flag = 1;
            elseif ExploreSequence(jj).map(1, ExploreSequence(jj).CurrentPosition) == 0
                ExploreSequence(jj).flag = 0;
            end
            ExploreSequence(jj).obsearvedMap = [ExploreSequence(jj).obsearvedMap ExploreSequence(jj).flag];
        end
        
    end
end




figure(4)
% axis equal;
font = 'Times';
figure('DefaultTextFontName', font, 'DefaultAxesFontName', font);
% title({'Available parking spot search with guarding strategy'},'fontsize',16);
hold on;
set(gca,'FontSize',16);
% axis equal;
xlim([0 50]);
ylim([-10 50]);
xticklabels({'0','10','20','30','40','50'});
yticklabels({'0','10','20','30','40','50','60'});
xlabel({'{\it x} [m]'}) 
ylabel({'{\it y} [m]'}) 



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
                if rand(1) > 1
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
                if rand(1) > 0
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
% for k = 1:3
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

if (ParkLotRowNum-1)*RowGap + 2 - Coordinate(ExploreSequence(i).NodeExplored(k+1),1) -(1+RowGap/2/ColGap-1)*ColGap+1.5 == UnavailCoordinate(j,2)
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



