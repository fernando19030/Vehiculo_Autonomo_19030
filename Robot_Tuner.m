%% Conexión a Robotat y Datos Pololu 3Pi+
Robotat = robotat_connect('192.168.50.200');
c_carro01 = robotat_3pi_connect(2);

Carro01 = robotat_get_pose(Robotat, 1, 'XYZ');% Pololu 3Pi+ pose

Obstaculo1 = robotat_get_pose(Robotat, 10, 'XYZ');%actuator pose

%% Parametros de Control

% Control de Stanley
k_stan = 0.25;

% PID orientación
kpO = 12.5;
kiO = 0.01;
kdO = 1;
DeltaO = 0;
delta_1 = 0;

% Acercamiento exponencial
v0 = 5;
alpha = 2;

%% Control
p_indice = 1;
DISTANCE_FROM_CENTER = 4.5;
WHEEL_RADIUS = 3.2/2;
tolerancia = 0.5;

while 1
    try
        x_y_z_gps = robotat_get_pose(Robotat, 1, 'XYZ');%actuator pose
        Obstaculo1 = robotat_get_pose(Robotat, 10, 'XYZ');%actuator pose
        
        x = x_y_z_gps(1); y = x_y_z_gps(2); 
        theta = deg2rad(x_y_z_gps(6));

        % Obtener el error de posición y la distancia a la siguiente posición en p
        e = [Obstaculo1(1) - x; Obstaculo1(2) - y];
        thetag = atan2(e(2), e(1));

        eP = norm(e);
        %distancia_siguiente_punto = norm(p(1),:))
        %if norm([xg; yg])-eP >=0.1; break;
        %end

        eO = thetag - theta;
        eO = atan2(sin(eO), cos(eO))

        % Control de velocidad lineal
%         kP = v0 * (1-exp(-alpha*eP^2)) / eP;
%         v = kP*eP;
         v = 0; 

        % Control de velocidad angular
        yaw = thetag - theta;
        yaw = atan2(sin(yaw), cos(yaw));
%         if (yaw < 0)
%             yaw = 2*pi + thetag;
%         end
%         delta = yaw + atan((k_stan*eP)/v);
        delta = yaw;
        
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
        robotat_3pi_set_wheel_velocities(c_carro01, vel_left, vel_right);
            
        if abs(eO) > deg2rad(5)
            vel_left = (v - w*DISTANCE_FROM_CENTER) / WHEEL_RADIUS;
            vel_right = (v + w*DISTANCE_FROM_CENTER) / WHEEL_RADIUS;
            robotat_3pi_set_wheel_velocities(c_carro01, vel_left, vel_right);
        end
        if abs(eO) < deg2rad(5)
           robotat_3pi_set_wheel_velocities(c_carro01, 0, 0);
        end

    catch
    end
end

%% Stop 
robotat_3pi_force_stop(c_carro01);