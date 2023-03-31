function values=costcal(a,b)
% Finds the eucliedean distance from 'a' to all points in 'b' 
% size of 'a' is 1 X 2 size of 'b' is n X 2, output values should be array
% of 'n'values
%% Write your code here

diff = b - a;
values = sqrt(sum(diff.^2));

end

