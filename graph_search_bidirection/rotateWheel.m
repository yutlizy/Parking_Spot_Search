function rotWheel = rotateWheel(wheel, angle)
[cx,cy] = centroid(wheel);
rotWheel = rotate(wheel, angle, [cx,cy]);
end