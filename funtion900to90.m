clear all;
clc;
close all;

%load("formation_path_init800_multiedgewithobs_50000_550_175_6.mat");
load("formation_path_init250_wij1930to1050_50000_6.mat");

step = 5;

dataset_traj_x = dataset_traj_x(1:step:end,:);
dataset_traj_y = dataset_traj_y(1:step:end,:);
frame = frame / step;

%save("formation_path_init800_multiedgewithobsstep10_50000_50_20_6.mat", 'dataset_traj_x', "dataset_traj_y", "dataset_traj_th", "dataset_label", "dataset_goal", "ob_temp", "frame")
%save("formation_path_init250_multiedgewithobsstep5930to1050_50000_90_25_6.mat", 'dataset_traj_x', "dataset_traj_y", "dataset_traj_th", "dataset_label", "dataset_goal", "ob_temp", "frame")
save("formation_path_init250_wij1step5930to1050_50000_90_25_6.mat", 'dataset_traj_x', "dataset_traj_y", "dataset_traj_th", "dataset_ob", "dataset_label", "dataset_goal", "step", "frame")


disp("success");

