%% Parámetros del sistema
ell = 0.01; % en m

%% Definición del sistema dinámico
% xi = [x; y; theta]  y  u = [v; omega]

% Campo vectorial del sistema dinámico
f = @(xi,u) [u(1)*cos(xi(3)); u(1)*sin(xi(3)); u(2)];

% Difeomorfismo para linealización por feedback
finv = @(xi,mu) [1,0; 0,1/ell] * [cos(xi(3)), -sin(xi(3)); sin(xi(3)), cos(xi(3))]' * [mu(1); mu(2)];

%% Matrices del sistema linealizado alrededor del punto de operación
A = zeros(2); 
B = eye(2); 

%% Obtención de la Trayectoria
selectraj = 3; % 1 D*, 2 cuadrado, 3 circulos, 4 DSD Toolbox

if selectraj == 1
    load('trajd_star');
    px = p(:, 1) - 380/2;
    py = p(:, 2) - 480/2;
elseif selectraj == 2
    load('Trayectoria_Cuadrada_Sim');
    px = p(:, 1);
    py = p(:, 2);
elseif selectraj == 3
    load('Trayectoria_Circular_sim');
    px = p(:, 1);
    py = p(:, 2);
elseif selectraj == 4
    load('Track_01.mat');
    refPose = data.ActorSpecifications.Waypoints; % Obtencion de trayectoria
    xRef = refPose(:,1) - 380/2; % Trayectoria en X
    yRef = refPose(:,2) - 480/2; % Trayectoria en Y
end

%% Parámetros de la simulación
t0 = 0; % tiempo inicial
tf = 10; % tiempo final

if selectraj == 1
    largo = length(p);
    dt = (tf-t0)/largo; % período de muestreo
elseif selectraj == 2
    largo = length(p);
    dt = (tf-t0)/largo; % período de muestreo
elseif selectraj == 3
    largo = length(p);
    dt = (tf-t0)/largo; % período de muestreo
elseif selectraj == 4
    dt = 0.001; % período de muestreo
end

N = (tf-t0)/dt; % número de iteraciones

%% Inicialización y condiciones iniciales
xi0 = [0; 0; 0];
u0 = [0; 0];
xi = xi0; % vector de estado 
u = u0; % vector de entradas
% Arrays para almacenar las trayectorias de las variables de estado,
% entradas y salidas del sistema
XI = zeros(numel(xi),N+1);
U = zeros(numel(u),N+1);
% Inicialización de arrays
XI(:,1) = xi0;
U(:,1) = u0;

%% Coordenadas de la meta
% Meta fija
xg = -2; % en m
yg = 3; % en m
thetag = 0; % en rad

% Trayectoria
seltraj = 1;
% t = 0:dt:tf;
% traj = 2*[cos(0.1*2*pi*t); sin(0.1*2*pi*t)];

if selectraj == 4

    % Calcular el vector de distancia
    distancematrix = squareform(pdist(refPose));
    distancesteps = zeros(length(refPose)-1,1);

    % Obtener la distancia individual entre puntos
    for i = 2:length(refPose)
        distancesteps(i-1,1) = distancematrix(i,i-1);
    end

    totalDistance = sum(distancesteps); % Distancia total recorrida
    distbp = cumsum([0; distancesteps]); % Distancia para cada punto
    gradbp = linspace(0,totalDistance,N); % Linializar la distancia

    % Linializar vectores X y Y basado en distancia 
    xRef2 = interp1(distbp,xRef,gradbp);
    yRef2 = interp1(distbp,yRef,gradbp);
    yRef2s = smooth(gradbp,yRef2); % Suavizado de los puntos
    xRef2s = smooth(gradbp,xRef2); % Suavizado de los puntos
    px = xRef2s;
    py = yRef2s;
end

p = [px, py];

traj = p;

%% Inicialización de variables para controladores
ctrlsel = 2; % selección de controlador a evaluar

% PID posición
kpP = 1;
kiP = 0.0001; 
kdP = 0.5;
EP = 0;
eP_1 = 0;

% PID orientación
kpO = 2*5;
kiO = 0.0001; 
kdO = 0;
EO = 0;
eO_1 = 0;

% Acercamiento exponencial
v0 = 2;
alpha = 0.5;

% Control de pose
k_rho = 3;
k_alpha = 8;
k_beta = -1.5;

% LQR 
Q = eye(2);
R = eye(2);
Klqr = 2*lqr(A,B,Q,R);

% LQR + Integradores (LQI)
Cr = eye(2);
Dr = zeros(2);
AA = [A, zeros(size(Cr')); Cr, zeros(size(Cr,1))];
BB = [B; Dr];
QQ = eye(size(A,1) + size(Cr,1)); QQ(3,3) = 10; QQ(4,4) = 10; 
Klqi = lqr(AA, BB, QQ, R);
ref = [xg; yg];
sigma = 0; 

%% Solución recursiva del sistema dinámico
for n = 0:N-1
    % Se obtiene el valor actual de la trayectoria a seguir (si aplica)
    if(seltraj)
        xg = traj(n+1,1);
        yg = traj(n+1,2);
    end
    
    % Se calcula el control
    switch ctrlsel
        % Control PID punto-a-punto/trayectorias
        case 1
            x = xi(1); y = xi(2); theta = xi(3);
            e = [xg - x; yg - y];
            thetag = atan2(e(2), e(1));
            
            eP = norm(e);
            eO = thetag - theta;
            eO = atan2(sin(eO), cos(eO));
            
            % Control de velocidad lineal
            eP_D = eP - eP_1;
            EP = EP + eP;
            v = kpP*eP + kiP*EP + kdP*eP_D;
            eP_1 = eP;
            
            % Control de velocidad angular
            eO_D = eO - eO_1;
            EO = EO + eO;
            w = kpO*eO + kiO*EO + kdO*eO_D;
            eO_1 = eO;
            
            % Se combinan los controladores
            u = [v; w];
            
        % Control PID punto-a-punto/trayectorias con acercamiento exponencial    
        case 2
            x = xi(1); y = xi(2); theta = xi(3);
            e = [xg - x; yg - y];
            thetag = atan2(e(2), e(1));
            
            eP = norm(e);
            eO = thetag - theta;
            eO = atan2(sin(eO), cos(eO));
            
            % Control de velocidad lineal
            kP = v0 * (1-exp(-alpha*eP^2)) / eP;
            v = kP*eP;
            
            % Control de velocidad angular
            eO_D = eO - eO_1;
            EO = EO + eO;
            w = kpO*eO + kiO*EO + kdO*eO_D;
            eO_1 = eO;
            
            % Se combinan los controladores
            u = [v; w];
            
        % Control de pose
        case 5
            x = xi(1); y = xi(2); theta = xi(3);
            e = [xg - x; yg - y];
            
            rho = norm(e);
            alpha = -theta + atan2(e(2), e(1));
            beta = -theta - alpha;
            
            v = k_rho*rho;
            w = k_alpha*alpha + k_beta*beta;
            
            u = [v;w];
            
        % Control LQR punto-a-punto  
        case 3
            x = xi(1); y = xi(2);
            e = [x - xg; y - yg];
            mu = -Klqr*e;
            u = finv(xi, mu);
            
        % Control LQI de trayectorias
        case 4
            x = xi(1:2);
            ref = [xg; yg];
            sigma = sigma + (Cr*x - ref)*dt;
            mu = -Klqi*[x; sigma];
            u = finv(xi, mu);
            
        otherwise
            v = 1;
            w = 1;
            u = [v; w];
    end
    
    % Se actualiza el estado del sistema mediante una discretización por 
    % el método de Runge-Kutta (RK4)
    k1 = f(xi, u);
    k2 = f(xi+(dt/2)*k1, u);
    k3 = f(xi+(dt/2)*k2, u);
    k4 = f(xi+dt*k3, u);
    xi = xi + (dt/6)*(k1+2*k2+2*k3+k4);
    
    % Se guardan las trayectorias del estado y las entradas
    XI(:,n+1) = xi;
    U(:,n+1) = u;
end

%% Animación y generación de figuras (NO modificar)
figure;
t = t0:dt:tf;
plot(t, XI', 'LineWidth', 1);
xlabel('$t$', 'Interpreter', 'latex', 'Fontsize', 16);
ylabel('$\mathbf{x}(t)$', 'Interpreter', 'latex', 'Fontsize', 16);
l = legend('$x(t)$', '$y(t)$', '$\theta(t)$', 'Location', 'best', ...
    'Orientation', 'vertical');
set(l, 'Interpreter', 'latex', 'FontSize', 12);
grid minor;

figure;
s = max(max(abs(XI(1:2,:))));
xlim(s*[-1, 1]+[-0.5, 0.5]);
ylim(s*[-1, 1]+[-0.5, 0.5]);
grid minor;
hold on;

q = XI(:,1);
x = q(1); y = q(2); theta = q(3);

if(seltraj)
    plot(traj(:,1), traj(:,2), 'k');
end

trajplot = plot(x, y, '--k', 'LineWidth', 1);

BV = [-0.1, 0, 0.1; 0, 0.3, 0];
IV = [cos(theta-pi/2), -sin(theta-pi/2); sin(theta-pi/2), cos(theta-pi/2)] * BV;
bodyplot = fill(IV(1,:) + x, IV(2,:) + y, [0.5,0.5,0.5]);

xlabel('$x$', 'Interpreter', 'latex', 'Fontsize', 16);
ylabel('$y$', 'Interpreter', 'latex', 'Fontsize', 16);
hold off;

for n = 2:N+1
    q = XI(:,n);
    x = q(1); y = q(2); theta = q(3);
    
    trajplot.XData = [trajplot.XData, x];
    trajplot.YData = [trajplot.YData, y];
    
    BV = [-0.1, 0, 0.1; 0, 0.3, 0];
    IV = [cos(theta-pi/2), -sin(theta-pi/2); sin(theta-pi/2), cos(theta-pi/2)] * BV;
    bodyplot.XData = IV(1,:) + x;
    bodyplot.YData = IV(2,:) + y;
  
    pause(dt);
end