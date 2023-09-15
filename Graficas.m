seltraj = 4;

if seltraj == 1
    load('D_star_uni.mat');
    load('trajd_star01.mat', 'p');
    px = p(:, 1);
    py = p(:, 2);
    l = length(px);
    
    q = XI;
    xu = q(1, 1:l) + 380/2; yu = q(2, 1:l) + 480/2;
    
    load('D_star_stan.mat');
    q = XI;
    x = q(1, 1:l) + 380/2; y = q(2, 1:l) + 480/2;
    
    
    px = p(:, 1);
    py = p(:, 2);

    figure(1);
    plot(px, py);
    title('Trayectoria D* Stanley-PID');
    hold on;
    plot(xu, yu);
    hold on;
    plot(x, y);
    xlabel('X');
    ylabel('Y');
    xlim([140 280]);
    ylim([200 400]);
    legend('trayectoria deseada', 'PID', 'Stanley-PID');
    
elseif seltraj == 2
    load('square_uni.mat');
    load('Trayectoria_Cuadrada_Sim.mat');
    px = px + 380/2;
    py = py + 480/2;
    l = length(px);
    
    q = XI;
    xu = q(1, 1:l) + 380/2; yu = q(2, 1:l) + 480/2;
    
    load('square_stan.mat');
    q = XI;
    x = q(1, 1:l) + 380/2; y = q(2, 1:l) + 480/2;
    
    px = px + 380/2;
    py = py + 480/2;

    figure(1);
    plot(px, py);
    title('Trayectoria Cuadrada Stanley-PID');
    hold on;
    plot(xu, yu);
    hold on;
    plot(x, y);
    xlabel('X');
    ylabel('Y');
    ylim([180 360]);
    xlim([150 300]);
    legend('trayectoria deseada', 'PID', 'Stanley-PID');
    
elseif seltraj == 3
    load('circle_uni.mat');
    load('Trayectoria_Circular_sim.mat');
    px = p(:, 1) + 380/2;
    py = p(:, 2) + 480/2;
    l = length(px);
    
    q = XI;
    xu = q(1, 1:l) + 380/2; yu = q(2, 1:l) + 480/2;
    
    load('circle_stan.mat');
    q = XI;
    x = q(1, 1:l) + 380/2; y = q(2, 1:l) + 480/2;
    
    px = px + 380/2;
    py = py + 480/2;

    figure(1);
    plot(px, py);
    title('Trayectoria Circular Stanley-PID');
    hold on;
    plot(xu, yu);
    hold on;
    plot(x, y);
    xlabel('X');
    ylabel('Y');
    ylim([220 260]);
    xlim([175 210]);
    legend('trayectoria deseada', 'PID', 'Stanley-PID');
    
elseif seltraj == 4
    load('track_uni.mat');
    load('Track_01.mat');
    px = px + 380/2;
    py = py + 480/2;
    l = length(px);
    
    q = XI;
    xu = q(1, 1:l) + 380/2; yu = q(2, 1:l) + 480/2;
    
    load('track_stan.mat');
    q = XI;
    x = q(1, 1:l) + 380/2; y = q(2, 1:l) + 480/2;
    
    px = px + 380/2;
    py = py + 480/2;

    figure(1);
    plot(px, py);
    title('Trayectoria DSD Stanley-PID');
    hold on;
    plot(xu, yu);
%     hold on;
%     plot(x, y);
    xlabel('X');
    ylabel('Y');
    ylim([200 450]);
    xlim([120 300]);
    legend('trayectoria deseada', 'PID', 'Stanley-PID');
    
end


