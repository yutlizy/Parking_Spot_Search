function Policy = PolicyGeneration(CurrentPosition,DrivingCost,WalkingCost,Coordinate,IndexSequence)
% PolicyNum = TotalParkSpot + 1 + (TotalParkSpot - AvaliParkSpot);
% Coordinate = coordinate_generate(RowNum, ColNum, [1,1], 1, 1);
% Coordinate(TotalParkSpot+1,:) = Coordinate(length(Coordinate(:,1)),:) + [0 1];
PolicyNum = length(IndexSequence) - CurrentPosition + 1; 
drive_cost = 0;
for i = 1:PolicyNum
%     StopPosition = CurrentPosition - (i - 1);

    if i == 1
        Policy(1).Action = [0];
%         Policy(1).Cost = DrivingCost*(i -1) + WalkingCost*(Coordinate(CurrentPosition,1) - 1 + Coordinate(CurrentPosition,2) - 1);
        Policy(1).Cost = DrivingCost*(i -1) + WalkingCost*((Coordinate(IndexSequence(CurrentPosition),1) - 1)^2 + (Coordinate(IndexSequence(CurrentPosition),2) - 1)^2)^0.5;
    else
        Policy(i).Action = [Policy(i-1).Action(1:end - 1) -1 0];
%         Policy(i).Cost = DrivingCost*(i -1) + WalkingCost*((Coordinate(IndexSequence(CurrentPosition + i - 1),1) - 1)^2 + (Coordinate(IndexSequence(CurrentPosition + i - 1),2) - 1)^2)^0.5;
%         Policy(i).Cost = DrivingCost*(i -1) + WalkingCost*((Coordinate(IndexSequence(CurrentPosition + i - 1),1) - 1) + (Coordinate(IndexSequence(CurrentPosition + i - 1),2) - 1));

        drive_cost = drive_cost + DrivingCost*((Coordinate(IndexSequence(CurrentPosition + i - 1),1) - Coordinate(IndexSequence(CurrentPosition + i - 2),1))^2 +(Coordinate(IndexSequence(CurrentPosition + i - 1),2) - Coordinate(IndexSequence(CurrentPosition + i - 2),2))^2)^0.5;

        Policy(i).Cost = drive_cost + WalkingCost*((Coordinate(IndexSequence(CurrentPosition + i - 1),1) - 1)^2 + (Coordinate(IndexSequence(CurrentPosition + i - 1),2) - 1)^2)^0.5;
% %  
%         drive_cost = drive_cost + DrivingCost*((Coordinate(IndexSequence(CurrentPosition + i - 1),1) - Coordinate(IndexSequence(CurrentPosition + i - 2),1))^2 +(Coordinate(IndexSequence(CurrentPosition + i - 1),2) - Coordinate(IndexSequence(CurrentPosition + i - 2),2))^2)^0.5;
% 
%         Policy(i).Cost = drive_cost + WalkingCost*((Coordinate(IndexSequence(CurrentPosition + i - 1),1) - 1) + (Coordinate(IndexSequence(CurrentPosition + i - 1),2) - 1));
%     else
%         Policy(i).Action = [Policy(i-1).Action(1:end - 1) 1 0];
% %         Policy(i).Cost = DrivingCost*TotalParkSpot + DrivingCost*(i -1 - TotalParkSpot) + WalkingCost*((Coordinate(i - 1 - TotalParkSpot + 1,1) - 1) + (Coordinate(i - 1 - TotalParkSpot + 1,2) - 1));
%         Policy(i).Cost = DrivingCost*TotalParkSpot + DrivingCost*(i -1 - TotalParkSpot) + WalkingCost*((Coordinate(i - 1 - TotalParkSpot + 1,1) - 1)^2 + (Coordinate(i - 1 - TotalParkSpot + 1,2) - 1)^2)^0.5;
    end
end
end