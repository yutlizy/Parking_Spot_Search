function PossibleAvailNum = PossibleNumGenerate(AvailNum)

if mod(AvailNum,2) == 0
    min_PossibleAvailNum = AvailNum/2;
else
    min_PossibleAvailNum = (AvailNum+1)/2;
end
max_PossibleAvailNum = AvailNum;

PossibleAvailNum = [min_PossibleAvailNum:max_PossibleAvailNum];

end