%% Obtencion de datos
Robotat = robotat_connect('192.168.50.200');
c_carro01 = robotat_3pi_connect(10);

Carro01 = robotat_get_pose(Robotat, 1, 'XYZ');% Robot pose

goal = [220, 380]; % Meta D*
%% Creación de mapa
walls = zeros(480,380);

walls(1,:) = 1; % Top wall
walls(end,:) = 1; % Bottom wall
walls(:,1) = 1; % Left wall
walls(:,end) = 1; % Right wall

Carrox = round((Carro01(1) + 3.8/2)*100);
Carroy = round((Carro01(2) + 4.8/2)*100);
initial_point = [Carrox, Carroy];

%% Selección de Trayectoria
seltraj = 1; % 1 D*, 2 cuadrado, 3 circulos

%% Generación de trayectoria
if seltraj == 1
    map = walls;
    dx = Dstar(map, 'inflate', 10); % numero sale de aprox de esto = 0.455*(500/10)
    dx.plan(goal); % A plan for moving to goal is generated
    p = dx.query(initial_point, 'animate');
elseif seltraj == 2
    load('Trayectoria_Cuadrada');
elseif seltraj == 3
    load('Trayectoria_Circular');
elseif seltraj == 4
    load('Track_01');
    p = data.ActorSpecifications.Waypoints; % Obtencion de trayectoria
end
%% PID
% Control de Stanley
k_stan = 0.5;

% PID orientación
kpO = 11;
kiO = 0.001;
kdO = 2;
DeltaO = 0;
delta_1 = 0;

% Acercamiento exponencial
v0 = 200; % 80 con tolerancia de 0.4, 120 para 0.2, 200 para 0.1, 400 para 0.05
alpha = 6;

%% Control
p_indice = 1;
DISTANCE_FROM_CENTER = 4.5;
WHEEL_RADIUS = 3.2/2;
tolerancia = 0.2;
real_traj_index = 1;
real_traj = zeros(2000, 6);

while 1
    try
        x_y_z_gps = robotat_get_pose(Robotat, 1, 'XYZ');%actuator pose
        real_traj(real_traj_index, :) = x_y_z_gps;
        
        x = x_y_z_gps(1); y = x_y_z_gps(2); 
        theta = deg2rad(x_y_z_gps(6));

        % Obtener el error de posición y la distancia a la siguiente posición en p
        e = [(p(p_indice, 1)/100 - 3.8/2) - x; (p(p_indice, 2)/100 - 4.8/2) - y];
        thetag = atan2(e(2), e(1));

        eP = norm(e);

        eO = thetag - theta;
        eO = atan2(sin(eO), cos(eO));

        % Control de velocidad lineal
        kP = v0 * (1-exp(-alpha*eP^2)) / eP;
        v = kP*eP;

        % Control de velocidad angular
        yaw = thetag - theta;
        yaw = atan2(sin(yaw), cos(yaw));
        delta = yaw + atan((k_stan*eP)/v);
        
        if delta > deg2rad(35)
            delta = deg2rad(35);
        elseif delta < deg2rad(-35)
            delta = deg2rad(-35);
        end
        
        delta_D = delta - delta_1; 
        DeltaO = DeltaO + delta;
        w = kpO*delta + kiO*DeltaO + kdO*delta_D;
        delta_1 = delta;

        % Se combinan los controladores
        u = [v; w];
        
        vel_left = (v - w*DISTANCE_FROM_CENTER) / WHEEL_RADIUS;
        vel_right = (v + w*DISTANCE_FROM_CENTER) / WHEEL_RADIUS;
        vel = [vel_left; vel_right];
        robotat_3pi_set_wheel_velocities(c_carro01, vel_left, vel_right);
            

        % Comprobar si se ha llegado al siguiente punto
        if eP <= tolerancia
            p_indice = p_indice + 1; % Avanzar al siguiente punto en p
        end

        % Comprobar si se ha llegado al último punto en p
        if p_indice == size(p,1)
            robotat_3pi_set_wheel_velocities(c_carro01, 0, 0);
            break; % Salir del bucle while
        end
        
        real_traj_index = real_traj_index + 1;
        
    catch
    end
end

%% Emergency Stop
robotat_3pi_force_stop(c_carro01);

%% Resultados
figure(1);
plot(p(:, 1), p(:, 2));
title('Trayectoria Deseada');
xlabel('X');
ylabel('Y');
ylim([0 480]);
xlim([0 380]);
hold on;

figure(2);
plot((real_traj(:, 1) + 3.8/2)*100, (real_traj(:, 2) + 4.8/2)*100);
title('Trayectoria Real');
xlabel('X');
ylabel('Y');
ylim([0 480]);
xlim([0 380]);
hold on;


if seltraj == 1
    save('trajd_star06.mat', 'real_traj', 'p');
elseif seltraj == 2
    save('traj_square06.mat', 'real_traj');
elseif seltraj == 3
    save('traj_circle06.mat', 'real_traj');
elseif seltraj == 4
    save('traj_DSD06.mat');
end
