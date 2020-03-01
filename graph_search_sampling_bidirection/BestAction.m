function [MinCost,OptimalAction,Num] = BestAction(Map,Policy,currentPosition,CurrentState)
% availableParkPostion = find(Map == 0);
MinCost = inf;
OptimalAction = [];
Num = 0;
PolicyNum = size(Policy);
Map = [1 Map];
% stopPosition = 
% for i = 1:length(availableParkPostion)
%     index = -availableParkPostion(i) + 1 + length(Map) + 1;
%     for j = 1:PolicyNum(2)
%         if length(Policy(j).Action) >= index && Policy(j).Action(index) == 0 && Policy(j).Cost < MinCost
%             MinCost = Policy(j).Cost;
%             OptimalAction = Policy(j).Action;
%             Num = j;
%         end
%     end
% end
for i = 1:PolicyNum(2)
    stopPosition = abs(sum(Policy(i).Action)) + currentPosition;
    if Map(stopPosition) == 0 && Policy(i).Cost < MinCost
        MinCost = Policy(i).Cost;
        OptimalAction = Policy(i).Action;
        Num = i;
    end
end