%  MATLAB Source Codes for the book "Cooperative Dedcision and Planning for
%  Connected and Automated Vehicles" published by Mechanical Industry Press
%  in 2020.
% 《智能网联汽车协同决策与规划技术》书籍配套代码
%  Copyright (C) 2020 Bai Li
%  2020.01.31
% ==============================================================================
%  第二章. 2.4.4小节. S-T图与A星搜索算法实现速度决策（即一种非常粗略的局部速度规划）并呈现结果
% ==============================================================================
%  备注：
%  1. st_graph_search_.penalty_for_inf_velocity取值不同可以体现出抢先通过或减速让行的差异，请读者自行尝试
%  2. 从动态效果来看会有离散化导致的微小误差，在工程实践中可通过设置碰撞缓冲区域来避免，但速度决策本身也只是
%     粗略的计算，因此有些误差也无妨.
% ==============================================================================
close all
clc

% % 参数设置
global vehicle_geometrics_ % 车辆轮廓几何尺寸
vehicle_geometrics_.vehicle_wheelbase = 2.8;
vehicle_geometrics_.vehicle_front_hang = 0.96;
vehicle_geometrics_.vehicle_rear_hang = 0.929;
vehicle_geometrics_.vehicle_width = 1.942;
vehicle_geometrics_.vehicle_length = vehicle_geometrics_.vehicle_wheelbase + vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_rear_hang;
vehicle_geometrics_.radius = hypot(0.25 * vehicle_geometrics_.vehicle_length, 0.5 * vehicle_geometrics_.vehicle_width);
vehicle_geometrics_.r2x = 0.25 * vehicle_geometrics_.vehicle_length - vehicle_geometrics_.vehicle_rear_hang;
vehicle_geometrics_.f2x = 0.75 * vehicle_geometrics_.vehicle_length - vehicle_geometrics_.vehicle_rear_hang;
global vehicle_kinematics_ % 车辆运动能力参数
vehicle_kinematics_.vehicle_v_max = 2.5;
vehicle_kinematics_.vehicle_a_max = 0.5;
vehicle_kinematics_.vehicle_phy_max = 0.7;
vehicle_kinematics_.vehicle_w_max = 0.5;
vehicle_kinematics_.vehicle_kappa_max = tan(vehicle_kinematics_.vehicle_phy_max) / vehicle_geometrics_.vehicle_wheelbase;
vehicle_kinematics_.vehicle_turning_radius_min = 1 / vehicle_kinematics_.vehicle_kappa_max;
global environment_scale_ % 车辆所在环境范围
environment_scale_.environment_x_min = -20;
environment_scale_.environment_x_max = 20;
environment_scale_.environment_y_min = -20;
environment_scale_.environment_y_max = 20;
environment_scale_.x_scale = environment_scale_.environment_x_max - environment_scale_.environment_x_min;
environment_scale_.y_scale = environment_scale_.environment_y_max - environment_scale_.environment_y_min;

% % 用于S-T图搜索的A星算法涉及的参数
global st_graph_search_
st_graph_search_.num_nodes_s = 80;
st_graph_search_.num_nodes_t = 100;
st_graph_search_.multiplier_H_for_A_star = 2.0;
st_graph_search_.penalty_for_inf_velocity = 4;

% % 导入既定算例以及静止障碍物分布情况
global vehicle_TPBV_ obstacle_vertexes_
load TaskSetup.mat
[x, y, theta, path_length, completeness_flag] = ProvideCoarsePathViaHybridAStarSearch();
st_graph_search_.resolution_s = path_length / st_graph_search_.num_nodes_s;

% % 布设移动障碍物、指定运动时域并给出移动物体的运动轨迹
global dynamic_obs
st_graph_search_.max_t = round(path_length * 2);
st_graph_search_.max_s = path_length;
st_graph_search_.resolution_t = st_graph_search_.max_t / st_graph_search_.num_nodes_t;
dynamic_obs = GenerateDynamicObstacles();
for ii = 1 : size(dynamic_obs,1)
    for jj = 1 : size(dynamic_obs,2)
        obs = dynamic_obs{ii,jj};
        fill(obs.x, obs.y, [125, 125, 125] ./ 255);
    end
end
drawnow;

% % S-T图搜索以及动态效果
[t,s] = SearchVelocityInStGraph(x, y, theta);
DemonstrateDynamicResult(x, y, theta, t, s);