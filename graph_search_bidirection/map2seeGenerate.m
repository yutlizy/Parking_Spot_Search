function map2see = map2seeGenerate(map,IndexSequence)

for j = 2:length(IndexSequence)
    map2see(j) = map(IndexSequence(j));
end

end

