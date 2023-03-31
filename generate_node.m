function [map, nodelocation]= generate_node(map,nnode)

% merge vertices of all obstacle
obsx=map.pgx{1};
obsy=map.pgy{1};
for i=2:length(map.pgx)
    obsx=[obsx NaN map.pgx{i}];
    obsy=[obsy NaN map.pgy{i}];
end
map.obsx=obsx;
map.obsy=obsy; 
% set nodelocation to all zero
nodelocation=zeros(nnode,2);
if(map.isgrown)
    for jj=1:nnode
        nodelocation(jj,2) = map.ystart;
    end
end
% generate nodes
n=1;
while (n<=nnode)
    % generate random two number in range of map's border
    %% Write your code here

    rx = rand * map.xend;
    ry = rand*(map.yend-map.ystart)+map.ystart;

    % if this node is not inside any obstacle
    %% Write your code here
    if ~inpolygon(rx, ry, obsx, obsy) %give inputs to inpolygon()
        nodelocation(n, :) = [rx ry];
        
        % add this location to nodelocation list


    end
    n=n+1;
end
hold on;
plot(nodelocation(:,1),nodelocation(:,2),'r*');
hold off;