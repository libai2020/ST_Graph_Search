%  MATLAB Source Codes for the book "Cooperative Dedcision and Planning for
%  Connected and Automated Vehicles" published by Mechanical Industry Press
%  in 2020.
% 《智能网联汽车协同决策与规划技术》书籍配套代码
%  Copyright (C) 2020 Bai Li
%  2020.01.31
% ==============================================================================
function obstacle_cell = GenerateDynamicObstacles()
global st_graph_search_
load DynObs
Nobs = size(dynamic_obs,2);
obstacle_cell = cell(st_graph_search_.num_nodes_t, Nobs);
for ii = 1 : Nobs
    dx = dynamic_obs{end,ii}.x(1) - dynamic_obs{1,ii}.x(1);
    dy = dynamic_obs{end,ii}.y(1) - dynamic_obs{1,ii}.y(1);
    for jj = 1 : st_graph_search_.num_nodes_t
        temp.x = dynamic_obs{1,ii}.x + dx / st_graph_search_.num_nodes_t * (jj - 1);
        temp.y = dynamic_obs{1,ii}.y + dy / st_graph_search_.num_nodes_t * (jj - 1);
        obstacle_cell{jj, ii} = temp;
    end
end
end