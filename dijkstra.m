function [Route, Cost] = dijkstra(exbigraph,exbiloc,startnode)
% Find path and cost to all other nodes in the graph
% Route keeps the optimal path to reach to all points from start
% Cost keeps all the 

graph=exbigraph; % graph(x,y) = 1 means there is an edge between nodes x and y & '0' for vice-versa
Loc=exbiloc;     
n=size(graph,1); % number of total nodes
Cost=inf(n,1);   % Initiating starting cost to infinite
curr=startnode;  % Staring the algorithm from start node
Route=cell(n,1); % Initializes the  empty routes to all nodes from start
Cost(curr)=0;    % Initiating the cost of start node to zero
Tabu=[curr];     % Storing the list of visited nodes
cRoute=[curr];   % Route for currently exploring node
i=1;
% for number of nodes
while i<n
    i=i+1;
    % take all possible node list
    T=1:n;
    % delete already visited node
    T(Tabu)=[];   
    % find connected node from current node which are not visited yet

    %% Write your code here
    ConnectedNodes = [];
    for ii=1:n
        if(graph(curr,ii) == 1)
            ConnectedNodes = [ConnectedNodes ii];
        end
    end
   
    %% Write your code here
    % calculate connected nodes euclid distance from current node and add
    % current nodes cost! 
    % below line is partially correct, u need to fill the second argument
    % to the function cost cal
    for j = 1:length(ConnectedNodes)
        cost=costcal(Loc(curr,:), Loc(ConnectedNodes(j),:))  + Cost(curr);
        if Cost(ConnectedNodes(j))>cost
            Cost(ConnectedNodes(j)) = cost;
            Route{ConnectedNodes(j)} = [cRoute ConnectedNodes(j)];
        end
    end
    

    %% Write your code here
    % if new finding coset is less than old cost then replace


    
    %% Write your code here
    % update new comming node rote if the new cost is more efficient
    
    
%% 
    % finding minimum costed node which is not visited yet in this iteration
    [mn cr]=min(Cost(T));
    %% Write your code here
    if isinf(mn)
         
        % Suggest the condition when the minimum cost of unvisited node
        % become infinite


    end
    %selecting the next node for exploration
    curr=T(cr);
    % select selected route as forbitten route to not to select this nodes
    % again
    cRoute=Route{curr};

    % Updating the table of visited nodes
    Tabu=[Tabu curr];
end
