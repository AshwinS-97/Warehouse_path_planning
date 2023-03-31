function [undirectedGraph,unedges]=generate_undirected_graph(map,nodelocation)
% takes map and produce undirected map and its adge set

% take number of nodes
nnode=size(nodelocation,1);
% set graph and edges as zero
undirectedGraph=zeros(nnode,nnode);
unedges=[];
%generate undirected graph

%% Write your code here
for i=1:nnode-1 
    for j=i+1:nnode
        
        
        % take each pair of node's location x and y
        x1 = [nodelocation(j,1), nodelocation(i,1)];
        y1 = [nodelocation(j,2) ,nodelocation(i,2)];
        % check this two nodes intersect any obstacle or not

        N = 100;
        % sample the points along the line 
        x_sampled = linspace(x1(1),x1(2),N);
        y_sampled = linspace(y1(1),y1(2),N);
        sampled_points = [x_sampled;y_sampled];

        %[xi,yi] = polyxpoly(x1,y1,map.obsx,map.obsy); %give inputs to polyxpoly()

        % if there is no intersection
        if ~any(checkOccupancy(map, sampled_points'))
            % connect this two nodes in graph
            undirectedGraph(i,j)=1;
            undirectedGraph(j,i)=1;
            % add this connection to edge set
            unedges = [unedges ; i , j];

        end
    end
end

% plot graph
figure 
show(map);
hold on;
for i=1:2:size(unedges,1)
    x1=[nodelocation(unedges(i,1),1);nodelocation(unedges(i,2),1)];
    y1=[nodelocation(unedges(i,1),2);nodelocation(unedges(i,2),2)]; 
    line(x1,y1);
end
hold off;