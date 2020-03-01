function regret = RegretCompute(Map,Policy,index,MinCost,OptimalAction,actual_flag)

% StopPosition = length(Map) - (find(Policy(index).Action == 0) - 1);
for ii = 1:length(Policy(index).Action)
    action(ii) = Policy(index).Action(ii);
end

StopPosition = length(Map) + sum(action);

is = isempty(strfind(OptimalAction,Policy(index).Action(1:end-1)));%% first element of strfind return value == 1
% if TimeStep == 1
%     Map = [Map 0];
if Map(StopPosition) == 0
    regret = Policy(index).Cost - MinCost;
elseif is == 0
    regret = 0;
elseif (index ~= 1) && (index <= length(Map))
    if Map(1:StopPosition) == ones(1,StopPosition)
        l = 1;
        while Map(StopPosition + l) == 1
            l = l + 1;
        end
        regret = Policy(index + l + 2*(StopPosition - 1)).Cost - MinCost;
    else
        l = 0;
        while Map(StopPosition - l) == 1
            l = l + 1;
        end
        regret = Policy(index + l).Cost - MinCost;
    end

elseif (index ~= 1) && (index > length(Map))
    regret = 0;
elseif index == 1
    regret = 1000;
end
% else
%     if Map(StopPosition) == 0
%         regret = Policy(index).Cost - MinCost;
%     elseif strifind(OptimalAction,Policy(index).Action) ~= []
%         regret = 0;
%     else
%         l = 1;
%         while Map(StopPosition + l) == 1
%             l = l + 1;
%         end
%         regret = Policy(index + l).Cost - MinCost;
%     end
% end
end