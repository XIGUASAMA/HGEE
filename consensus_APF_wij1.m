function [ output_args ] = consensus_APF( input_args )
%%% Created: 2023-11-1
%%% Last modified: 2024-8-19
%%% Author: Peiqiao Shang

close all;
fol_num=5;       % 5 followers
N=6;             % 5 followers and 1 leader
countmax=1500;   % Largest cycle count number
dt=0.1;          % Control and communication period
gama=0.5;
beta=1;         
K0=1;
KN=0.2;         % Four control parameters

%goal=[10 10] + (rand(1,2))*2;   % Destination point

dataset_traj_x = [];
dataset_traj_y = [];
dataset_traj_th = [];
dataset_label = [];
dataset_goal = [];
dataset_ob = [];

init_frame = 230;
frame = 700;
tolerant_maxcount = 1050;
tolerant_mincount = init_frame + frame;

% x and y speed :m/s, angular velocity: rad/s, acceleration: m/s^2 and rad/s^2
Kinematic=[0.5;0.5;0.5;0.5;0.5;0.5];
%% Adjacent matrix
% A=[0 1 0 0 0 0;     % a(ij)
%    0 0 0 1 0 0;
%    0 0 0 0 1 0;
%    0 0 0 0 0 1;
%    0 0 0 0 0 1;
%    0 0 0 0 0 0];

% A=[0 1 0 0 0 0;
%    1 0 1 0 0 1;
%    0 1 0 1 0 0;
%    0 0 1 0 1 1;
%    0 0 0 1 0 0;
%    0 1 0 1 0 0];
% 
% A=[0 1 1 1 1 1;
%    1 0 1 1 1 1;
%    1 1 0 1 1 1;
%    1 1 1 0 1 1;
%    1 1 1 1 0 1;
%    1 1 1 1 1 0];

iter = 1;
max_sample = 100000;
while iter <= max_sample

%disp(iter);
goal=[23 23] + (rand(1,2))*4;   % Destination point

% 生成一个N*N的随机01矩阵
A = randi([0, 1], N, N);
% 将对角线上的元素置为零
A = A - diag(diag(A));
% 将矩阵转换为对称矩阵
A = triu(A) + triu(A, 1)';

if checkConnectivity(A)
    %disp("pass")
else
    %disp("fail for A");
    continue;
end

 %% Initial position matrix
%         init_f=[0 -0 pi/4;%%%[x y th]
%                 0 -3 0; 
%                 0 -6 0;
%                 0 -9 0;
%                 0 -12 0;];
 
 
     
      init_f_xy = (-1*rand(N,2))*2;
      init_f_th = (rand(N,1)-0.5)*2*pi;
      init_f = [init_f_xy init_f_th];


    %  init_f=[-1 1 pi/4;%%%[x y th]
    %          1 1 pi/4; 
    %          2 0 pi/4;
    %          1 -1 pi/4;
    %          -1 -1 pi/4;
    %          0 0 pi/4];
    % 
    % rows_init_f = size(init_f,1);
    % random_numbers = (rand(rows_init_f, 2)-0.5) * 2;
    % init_f(:, 1:2) = init_f(:, 1:2) + random_numbers;
    % init_f_xy = init_f(:, 1:2);
    % init_f_th = init_f(:,3);


    distance_check = 1;
    for i = 1:N
        for j = i+1:N
            dist = norm(init_f_xy(i,:) - init_f_xy(j,:)); % 计算第i行和第j行的欧式距离
            if dist < 0.7
                %disp("fail for initial distance")
                distance_check = 0;
                break
            end
        end
    end
    
    if distance_check == 0
        continue
    end
     
    pose_x=init_f(:,1);
    pose_y=init_f(:,2);
    pose_th=init_f(:,3);
    pose_x(:,2)=init_f(:,1);
    pose_y(:,2)=init_f(:,2);
    pose_th(:,2)=init_f(:,3);

    %ob_temp=[19 21; 30 30];
    %ob_temp=[];
    
    %% generate ob_temp
    x_init = 6+rand(1);
    x_end = 8+rand(1);
    y_init = 6+rand(1);
    y_end = 8+rand(1);
    xy_step = 0.03;
    ob_temp = [];
    dataset_ob_one = [];
    for i = x_init:xy_step:x_end
        for j = y_init:xy_step:y_end
            kkk = [i j];
            ob_temp = [ob_temp; kkk];
        end
    end
    dataset_ob_one = [dataset_ob_one x_init y_init x_end-x_init y_end-y_init];


    x_init = 12+rand(1);
    x_end = 14+rand(1);
    y_init = 17+rand(1);
    y_end = 19+rand(1);
    xy_step = 0.03;
    for i = x_init:xy_step:x_end
        for j = y_init:xy_step:y_end
            kkk = [i j];
            ob_temp = [ob_temp; kkk];
        end
    end
    dataset_ob_one = [dataset_ob_one x_init y_init x_end-x_init y_end-y_init];

    % x_init = 16+rand(1);
    % x_end = 18+rand(1);
    % y_init = 16+rand(1);
    % y_end = 18+rand(1);
    % xy_step = 0.03;
    % for i = x_init:xy_step:x_end
    %     for j = y_init:xy_step:y_end
    %         kkk = [i j];
    %         ob_temp = [ob_temp; kkk];
    %     end
    % end
    

    x_init = 19+rand(1);
    x_end = 21+rand(1);
    y_init = 14+rand(1);
    y_end = 16+rand(1);
    xy_step = 0.03;
    for i = x_init:xy_step:x_end
        for j = y_init:xy_step:y_end
            kkk = [i j];
            ob_temp = [ob_temp; kkk];
        end
    end
    dataset_ob_one = [dataset_ob_one x_init y_init x_end-x_init y_end-y_init];



    %% consensus relative position 
%     delta_x=2*[-2 -2 -1 -1 0];    
%     delta_y=2*[1 -1 1 -1 0];  %relative position between leader and follwers

    delta_x = 1.5*[-1 1 2 1 -1 0];
    delta_y = 1.5*[1 1 0 -1 -1 0];
     
    V_x(:,1)=[0;0;0;0;0;0];
    V_y(:,1)=[0;0;0;0;0;0]; % velocity matrix
    k=1;    % counting number
    d_max=2;
    detect_R=1; %%obstacle detection range

    %% edge weighted matrix
    edge_w=[];
    sum_weight=[0;0;0;0;0;0];
                
    %% Begin to run
    
    if max_sample < 1000
        disp("Please check the max_iter and the save model parameter")
    end

    for count=1:countmax
        %disp(count);
        k=k+1;
%         store the last time velocity
        %%%Calculate attraction from goal and put it on leader velocity
        distance=sqrt((goal(1)-pose_x(N,k))^2+(goal(2)-pose_y(N,k))^2);
        th=atan2(goal(2)-pose_y(N,k),goal(1)-pose_x(N,k));
        if distance>d_max
            distance=d_max;
        end
        V_x(N,k+1)=KN*distance*cos(th);
        V_y(N,k+1)=KN*distance*sin(th);

        %% Calculate the agents velocity
            kk=0;
            for j=1:N
                if j~=i
                    kk=kk+1;
                    obs_pose(kk,1)=pose_x(j,k);
                    obs_pose(kk,2)=pose_y(j,k);
                end
            end
            ob_pose=[obs_pose;ob_temp];
        %%%ob_pose=ob_temp;
        repulsion=compute_repulsion([pose_x(N,k),pose_y(N,k)],ob_pose,detect_R);        
        % Put obstacle repulsion to leader
        V_x(N,k+1)=V_x(N,k+1)+beta*repulsion(1);
        V_y(N,k+1)=V_y(N,k+1)+beta*repulsion(2);
%         [V_x(N,k+1),V_y(N,k+1)]=DynamicWindowApproach(pose_x(N,k),pose_y(N,k),ob_pose,V_x(N,k+1),V_y(N,k+1));
        % When the local minimum appears, give an random error
        if(distance>1&&abs(V_x(N,k+1))<=0.1&&abs(V_y(N,k+1))<=0.1)
            V_x(N,k+1)=-1+2*rand(1);
            V_y(N,k+1)=-1+2*rand(1);
        end



        %% My Control Law to leader velocity


        %W = zeros(N,N);
%%
        % Calculate followers' velocity
        for i=1:fol_num        
            sum_delta_x=0;
            sum_delta_y=0;
            sum_edge_weight=0;
            for j=1:N       %%One-order Consensus control
                if A(i,j)==1
                    %w_ij=2-exp(-((pose_x(j,k-1)-pose_x(i,k)-(delta_x(j)-delta_x(i)))^2+(pose_y(j,k-1)-pose_y(i,k)-(delta_y(j)-delta_y(i)))^2)); %edge weighted calculation 
                    w_ij = 1;
                    %W(i,j) = w_ij;
                    sum_delta_x=sum_delta_x+A(i,j)*w_ij*((pose_x(j,k-1)-pose_x(i,k))-(delta_x(j)-delta_x(i)));
                    sum_delta_y=sum_delta_y+A(i,j)*w_ij*((pose_y(j,k-1)-pose_y(i,k))-(delta_y(j)-delta_y(i)));
                    sum_edge_weight=sum_edge_weight+w_ij;
                end
            end
            %disp(W);
            edge_w(i,k)=sum_edge_weight;
            % Store the weight error into sum_weight matrix
            if edge_w(i,k)>edge_w(i,k-1)&&k>2
                sum_weight(i,k)=sum_weight(i,k-1)+abs(edge_w(i,k)-edge_w(i,k-1));
            else
                sum_weight(i,k)=sum_weight(i,k-1);
            end
            if mod(k,100)==1 % reset matrix to 0 after period 10s
                sum_weight(i,k)=0;
            end
            
            distance=sqrt(sum_delta_x^2+ sum_delta_y^2);
            th=atan2(sum_delta_y, sum_delta_x);
            if distance>d_max
                distance=d_max;
            end
            V_x(i,k+1)=K0*V_x(N,k)+gama*distance*cos(th); % ideal velocity without repulsion from obstalcle
            V_y(i,k+1)=K0*V_y(N,k)+gama*distance*sin(th);
            
            out=confine([V_x(i,k) V_y(i,k)],[V_x(i,k+1) V_y(i,k+1)],Kinematic,0.1);
%             out=[V_x(i,k+1) V_y(i,k+1)];
            V_x(i,k+1)=out(1);
            V_y(i,k+1)=out(2);

           %%%Consider repulsion among agents and store the pose to obs_pose
            kk=0;
            for j=1:N
                if j~=i
                    kk=kk+1;
                    obs_pose(kk,1)=pose_x(j,k);
                    obs_pose(kk,2)=pose_y(j,k);
                end
            end
            ob_pose=[obs_pose;ob_temp];
            repulsion=compute_repulsion([pose_x(i,k),pose_y(i,k)],ob_pose,detect_R);        
            % put repulsion from obstacle on the robot velocity
            V_x(i,k+1)=V_x(i,k+1)+beta*repulsion(1);
            V_y(i,k+1)=V_y(i,k+1)+beta*repulsion(2);
            
        end
        
        % update the position and calculate error between prediction and
        % real position
        for i=1:N
            out=confine([V_x(i,k) V_y(i,k)],[V_x(i,k+1) V_y(i,k+1)],Kinematic,0.1);
%             out=[V_x(i,k+1) V_y(i,k+1)];
            V_x(i,k+1)=out(1);
            V_y(i,k+1)=out(2);
            pose_x(i,k+1)=pose_x(i,k)+dt*V_x(i,k+1);
            pose_y(i,k+1)=pose_y(i,k)+dt*V_y(i,k+1);
            pose_th(i,k+1)=atan2(V_y(i,k+1),V_x(i,k+1));
        end
        
        %% ====Animation====

        % area = compute_area(pose_x(N,k+1),pose_y(N,k+1),N+1);
        % hold off;
        % ArrowLength=0.5;% 
        % for j=1:N
        %     quiver(pose_x(j,k+1),pose_y(j,k+1),ArrowLength*cos(pose_th(j,k+1)),ArrowLength*sin(pose_th(j,k+1)),'*k');hold on;
        %     if j==N
        %         state=2;
        %     else
        %         state=1;
        %     end
        %     draw_circle (pose_x(j,k+1),pose_y(j,k+1),0.25,state);hold on;
        % end
        % for i=1:N
        %     for j=1:N
        %         if A(i,j)==1
        %             draw_arrow([pose_x(j,k+1),pose_y(j,k+1)],[pose_x(i,k+1),pose_y(i,k+1)], .2);hold on;
        %         end
        %     end
        % end
        % if size(ob_temp)~=[0 0]
        %     plot(ob_temp(:,1),ob_temp(:,2),'Xk','LineWidth',2);hold on;
        % end
        % axis(area);
        % grid on;
        % drawnow;    

        %% Judge whether arrived
        now_1=[pose_x(1,k+1),pose_y(1,k+1)];
        now_N=[pose_x(N,k+1),pose_y(N,k+1)];
        if norm(now_N-goal)<1 && norm(now_1-goal)<4
            if length(pose_x(:)) / N >= frame && count <= tolerant_maxcount && count > tolerant_mincount
                %disp('Arrive Goal!!');
                iter = iter + 1
                if mod(iter, 1000) == 0
                    disp(iter);
                end
                dataset_traj_x = [dataset_traj_x; pose_x(:,init_frame + 1:init_frame + frame)'];
                dataset_traj_y = [dataset_traj_y; pose_y(:,init_frame + 1:init_frame + frame)'];
                dataset_traj_th = [dataset_traj_th; init_f_th'];
                dataset_ob = [dataset_ob; dataset_ob_one];
                dataset_label = [dataset_label; A];
                dataset_goal = [dataset_goal; goal];
                break;
            end
        end
    
    end
        %% save traj and label
    %dataset_traj = [dataset_traj; pose_x'];
    %dataset_label = [dataset_label; ]
end 
    color='mgbkrc'; %%%corresponding to 6 colors
    type=[2,1,0.5,0.5,2,2];%%%different line type
%     xlswrite('attmse.xlsx',attmse);
    %% Draw diagram
    figure                               % Draw the path record of formation 
    for i=1:N
        plot(pose_x(i,:),pose_y(i,:),color(1,i),'LineWidth',2);
        hold on
    end
    for i=1:N-1
        plot(pose_x(i,1),pose_y(i,1),'bp','color',color(1,i),'LineWidth',1);
        hold on
    end
    plot(pose_x(N,1),pose_y(N,1),'*','color',color(1,N),'LineWidth',1);
    hold on
    for i=1:N-1
        plot(pose_x(i,k),pose_y(i,k),'m^','color',color(1,i),'LineWidth',2);
        hold on
    end
    plot(pose_x(N,k),pose_y(N,k),'o','color',color(1,N),'LineWidth',2);
    hold on
    if size(ob_temp)~=[0 0]
        plot(ob_temp(:,1),ob_temp(:,2),'Xk','LineWidth',2);hold on;
    end
    grid on;
    xlabel('x');
    ylabel('y');
    legend('Follower 1','Follower 2','Follower 3','Follower 4','Follower 5','Leader');
    xlabel('x(m)');
    ylabel('y(m)');
    title('Formation Consensus');

    %add if norm(now_N-goal)<1 && norm(now_1-goal)<4 in line 336
    %make w_ij = 1 -----change in 1.9
    save("formation_path_init250_wij1930to1050_100000_6.mat", 'dataset_traj_x', "dataset_traj_y", "dataset_traj_th", "dataset_ob", "dataset_label", "dataset_goal", "frame")
    disp(1);

end
