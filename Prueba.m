% =========================================================================
% MT3005: métodos de control para el uniciclo
% =========================================================================

%% Parámetros del sistema
ell = 0.01; % en m
L = 2.65;

load('Prueba.mat');
% load('Straight.mat');
refPose = data.ActorSpecifications.Waypoints;

xRef = refPose(:,1);
yRef = refPose(:,2);

%% Definición del sistema dinámico
% xi = [x; y; theta]  y  u = [v; omega]

% Campo vectorial del sistema dinámico
f = @(xi,u) [u(1)*cos(xi(3)); u(1)*sin(xi(3)); u(2)];

% Difeomorfismo para linealización por feedback
finv = @(xi,mu) [1,0; 0,1/ell] * [cos(xi(3)), -sin(xi(3)); sin(xi(3)), cos(xi(3))]' * [mu(1); mu(2)];

%% Matrices del sistema linealizado alrededor del punto de operación
A = zeros(2); 
B = eye(2); 

%% Parámetros de la simulación
dt = 0.001; % período de muestreo
t0 = 0; % tiempo inicial
tf = 10; % tiempo final
N = (tf-t0)/dt; % número de iteraciones

%% Inicialización y condiciones iniciales
% xi0 = [xRef2(1); yRef(2); 0];
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
% Trayectoria
seltraj = 1;

% calculate distance vector
distancematrix = squareform(pdist(refPose));
distancesteps = zeros(length(refPose)-1,1);
for i = 2:length(refPose)
    distancesteps(i-1,1) = distancematrix(i,i-1);
end
totalDistance = sum(distancesteps); % Total distance travelled
distbp = cumsum([0; distancesteps]); % Distance for each waypoint
gradbp = linspace(0,totalDistance,N); % Linearize distance

% linearize X and Y vectors based on distance
xRef2 = interp1(distbp,xRef,gradbp);
yRef2 = interp1(distbp,yRef,gradbp);
yRef2s = smooth(gradbp,yRef2); % smooth waypoints
xRef2s = smooth(gradbp,xRef2); % smooth waypoints

% xi0 = [xRef2(1); yRef2(1); 0];
% xi = xi0;
%% Inicialización de variables para controladores
ctrlsel = 1; % selección de controlador a evaluar

% PID posición
kpP = 1;
kiP = 0.0001; 
kdP = 0.5;
EP = 0;
eP_1 = 0;

% PID orientación
kpO = 11;
kiO = 0.001; 
kdO = 2;
% EO = 0;
% eO_1 = 0;
DeltaO = 0;
delta_1 = 0;

% Acercamiento exponencial
v0 = 8;
alpha = 0.5;

% Stanley
k_stan = 0.5;

%% Solución recursiva del sistema dinámico
for n = 0:N-1
    % Se obtiene el valor actual de la trayectoria a seguir (si aplica)
    if(seltraj)
        xg = xRef2(n+1);
        yg = yRef2(n+1);
    end
    
    % Se calcula el control
    % Control PID punto-a-punto/trayectorias
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
    plot(xRef2, yRef2, 'k');
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