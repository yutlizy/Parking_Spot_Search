function route = RouteGenerate(RowNum, ColNum, RowTraversalSequence)
RowTraversalSequenceNum = size(RowTraversalSequence);
for i = 1:RowTraversalSequenceNum(1)
    route(i).RowTraversalSequence = RowTraversalSequence(i,:);
end
Table = 1000*ones(RowNum,ColNum);
for i = 1:RowNum
    if mod(i,2) ~= 0
        Table(i,:) = [(i-1)*ColNum + 1 : i*ColNum];
    else
        Table(i,:) = [i*ColNum : -1 : i*ColNum - ColNum + 1];
    end
end

for i = 1:RowTraversalSequenceNum(1)
    route(i).TraversalSequence = [];
    CurrentRow = 1;
    ConnectNode = [];
    for j = 1:length(route(i).RowTraversalSequence)
        if CurrentRow ~= route(i).RowTraversalSequence(j) && mod(j,2) ~= 0
            if route(i).RowTraversalSequence(j) > CurrentRow + 1
                for k = 1:(route(i).RowTraversalSequence(j) - CurrentRow -1)
                    ConnectNode = [ConnectNode Table(RowNum + 1 - CurrentRow - k,ColNum)];
                end
            elseif route(i).RowTraversalSequence(j) == CurrentRow + 1
                ConnectNode = [];
            elseif route(i).RowTraversalSequence(j) < CurrentRow - 1
                for k = 1:(CurrentRow - route(i).RowTraversalSequence(j) -1)
                    ConnectNode = [ConnectNode Table(RowNum + 1 - CurrentRow + k,ColNum)];
                end
            elseif route(i).RowTraversalSequence(j) == CurrentRow - 1
                ConnectNode = [];
            end
            route(i).TraversalSequence = [route(i).TraversalSequence ConnectNode flip(Table(RowNum + 1 - route(i).RowTraversalSequence(j),:))];
            CurrentRow = route(i).RowTraversalSequence(j); 
            ConnectNode = [];
        elseif CurrentRow ~= route(i).RowTraversalSequence(j) && mod(j,2) == 0
            if route(i).RowTraversalSequence(j) > CurrentRow + 1
                for k = 1:(route(i).RowTraversalSequence(j) - CurrentRow - 1)
                    ConnectNode = [ConnectNode Table(RowNum + 1 - CurrentRow - k,1)];
                end
            elseif route(i).RowTraversalSequence(j) == CurrentRow + 1
                ConnectNode = [];
            elseif route(i).RowTraversalSequence(j) < CurrentRow - 1
                for k = 1:(CurrentRow - route(i).RowTraversalSequence(j) - 1)
                    ConnectNode = [ConnectNode Table(RowNum + 1 - CurrentRow + k,1)];
                end
            elseif route(i).RowTraversalSequence(j) == CurrentRow - 1
                ConnectNode = [];
            end
            route(i).TraversalSequence = [route(i).TraversalSequence ConnectNode Table(RowNum + 1 - route(i).RowTraversalSequence(j),:)];
            CurrentRow = route(i).RowTraversalSequence(j); 
            ConnectNode = [];
        elseif CurrentRow == route(i).RowTraversalSequence(j) && mod(j,2) ~= 0
            route(i).TraversalSequence = [route(i).TraversalSequence flip(Table(RowNum + 1 - route(i).RowTraversalSequence(j),:))];
            CurrentRow = route(i).RowTraversalSequence(j); 
            ConnectNode = [];
        elseif CurrentRow == route(i).RowTraversalSequence(j) && mod(j,2) == 0
            route(i).TraversalSequence = [route(i).TraversalSequence Table(RowNum + 1 - route(i).RowTraversalSequence(j),:)];
            CurrentRow = route(i).RowTraversalSequence(j); 
            ConnectNode = [];
        end
    end
end
for i = 1:RowTraversalSequenceNum(1)
    if route(i).RowTraversalSequence(1) ~= 1
        route(i).TraversalSequence = [RowNum*ColNum+1 Table(RowNum,ColNum) route(i).TraversalSequence];
    else
        route(i).TraversalSequence = [RowNum*ColNum+1 route(i).TraversalSequence];
    end
    if ismember(route(i).TraversalSequence(end),route(i).TraversalSequence(1:end-1))
        route(i).TraversalSequence(end) = [];
    end
end