function Map4Plan = MapGenerate(Sequence,AvailNum,UnAvailNum)
sameIndex = [];

for j = 1:length(Sequence)-1
    [val,pos]=intersect(Sequence(j+1:end),Sequence(j));
    if isempty(val) == 0
        sameIndex = [sameIndex; [j j+pos]];
    end
end
sameIndexNum = size(sameIndex);
possible = [];
if isempty(sameIndex) == 1
    Map4Plan = unique_perms([zeros(1,AvailNum),ones(1,UnAvailNum)]);
else
    if sameIndexNum(1) <= UnAvailNum
        for i = 1:sameIndexNum(1)-1
            possible = [possible;unique_perms([zeros(1,i),ones(1,sameIndexNum(2)-i)])];
        end
        possible = [possible;zeros(1,(sameIndexNum(2)));ones(1,(sameIndexNum(2)))];
    else
        for i = 1:UnAvailNum
            possible = [possible;unique_perms([zeros(1,sameIndexNum(2)-i),ones(1,i)])];
        end
        possible = [possible;zeros(1,(sameIndexNum(2)))];
    end
    Map4Plan = [];
    possibleNum = size(possible);
    
    for k = 1:possibleNum(1)
        
        ZeroNum = sum(possible(k,:) == 0);
        OneNum = sum(possible(k,:) == 1);
        OtherElement = unique_perms([ones(1,UnAvailNum-OneNum),zeros(1,AvailNum-ZeroNum)]);
        SequenceTemp = 10*ones(1,length(Sequence)-sameIndexNum(1)*sameIndexNum(2));
        
        ll = 1;
        indexTemp = [];
        for l = 1:length(Sequence)
            if ismember(l,sameIndex) == 0
                SequenceTemp(ll) = Sequence(l);
                ll = ll + 1;
                indexTemp = [indexTemp l];
            end
        end
        OtherElementNum = size(OtherElement);
        Map4PlanTemp = 10*ones(OtherElementNum(1),length(Sequence));
        %     OtherElementNum = size(OtherElement);
        for m = 1:OtherElementNum(1)
            for r = 1:sameIndexNum(1)
                Map4PlanTemp(m,[sameIndex(r,:)]) = possible(k,r);
            end
            Map4PlanTemp(m,[indexTemp]) = OtherElement(m,:);
        end
        Map4Plan = [Map4Plan;Map4PlanTemp];
    end
end

end