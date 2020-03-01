function coordinate = coordinate_generate(numRow, numCol, origin, row_gap, col_gap)

coordinate = ones(numRow*numCol,2);


for i = 1:numRow*numCol
    coordinate(i,1) = origin(1,1) + (ceil(i/numCol) - 1)*row_gap;
    if mod(i,numCol) ~= 0
        coordinate(i,2) = origin(1,2) + (mod(i,numCol) - 1)*col_gap;
    else
        coordinate(i,2) = origin(1,2) + (numCol - 1)*col_gap;
    end    
end
aa = {};
j = 0;
for i = 1:numRow*numCol
    if mod(coordinate(i,1),2) == 0 
        aa{end+1} = [coordinate(i,:)];
        
    elseif isempty(aa) == 0 %i > numCol 
        
        aa = flip(aa);
        for k = (i - numCol):(i - 1)
            coordinate(k,:) = aa{k - numCol - j*numCol};
        end
        aa = {};
        j = j + 2;
    end
    
end

end