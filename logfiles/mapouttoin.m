function map_out =  mapouttoin( bounds,  input)
% Maps the input which is in between bounds(1) - bounds(2) to 
% output which should be in between bounds(3) - bounds(4)
slope = (bounds(4)-bounds(3))/(bounds(2)-bounds(1));
inds1 = (input > bounds(2));
map_out(inds1) = bounds(4)*ones(sum(inds1),1);
inds2 = (input < bounds(2));
map_out(inds2) = bounds(3)*ones(sum(inds2),1);
inds = ~(inds1 & inds2);
map_out(inds) = bounds(3) + (input(inds)-bounds(1)).*slope;
end
