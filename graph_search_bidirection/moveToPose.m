function shape = moveToPose(shape, pose)

shape = rotate( ...
    translate(shape, [pose(1), pose(2)]), pose(3), [pose(1), pose(2)] );
end
