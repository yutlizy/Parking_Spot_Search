function PositionUpdate = PositionUpdate(CurrentPosition,Action,RowGap,ColGap)
if Action == 1 % 1 left; 
    PositionUpdate(1,1) = CurrentPosition(1,1) - ColGap;
    PositionUpdate(1,2) = CurrentPosition(1,2);    
elseif Action == 2 % 2 right; 
    PositionUpdate(1,1) = CurrentPosition(1,1) + ColGap;
    PositionUpdate(1,2) = CurrentPosition(1,2);
elseif Action == 3 % 3 up;
    PositionUpdate(1,1) = CurrentPosition(1,1);
    PositionUpdate(1,2) = CurrentPosition(1,2) + RowGap;
elseif Action == 4 % 4 down;
    PositionUpdate(1,1) = CurrentPosition(1,1);
    PositionUpdate(1,2) = CurrentPosition(1,2) - RowGap;
else % 5 stop
    PositionUpdate(1,1) = CurrentPosition(1,1);
    PositionUpdate(1,2) = CurrentPosition(1,2);
end
end