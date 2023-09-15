% =========================================================================
% MT3005: simulación de un robot diferencial
% =========================================================================
%% Parámetros del sistema
r = (195/1000)/2; % radio de las ruedas
ell = 381/1000; % distancia entre ruedas

%% Definición del sistema dinámico
% xi = [x; y; theta]  y  u = [phiR; phiL]
% f = @(xi,u) [ (r/2)*(u(1)+u(2))*cos(xi(3)); 
%               (r/2)*(u(1)+u(2))*sin(xi(3)); 
%                    (r/(2*ell))*(u(1)-u(2)) ];

f = @(xi,u) [u(1)*cos(xi(3)); u(1)*sin(xi(3)); u(2)];

%% Parámetros de la simulación
dt = 0.001; % período de muestreo
t0 = 0; % tiempo inicial
tf = 2; % tiempo final
K = (tf-t0)/dt; % número de iteraciones

%% Inicialización y condiciones iniciales
xi0 = [0; 0; 0];
u0 = [0; 0];
xi = xi0; % vector de estado 
u = u0; % vector de entradas
% Arrays para almacenar las trayectorias de las variables de estado,
% entradas y salidas del sistema
XI = zeros(numel(xi), K+1);
U = zeros(numel(u), K+1);
% Inicialización de arrays
XI(:,1) = xi0;
U(:,1) = u0;

%% Solución recursiva del sistema dinámico
for k = 1:K
    % Se definen las entradas
    phiR = 20;
    phiL = 10;
    u = [phiR; phiL];
    
    % Se actualiza el estado del sistema mediante una discretización por 
    % el método de Runge-Kutta (RK4)
    k1 = f(xi, u);
    k2 = f(xi+(dt/2)*k1, u);
    k3 = f(xi+(dt/2)*k2, u);
    k4 = f(xi+dt*k3, u);
    xi = xi + (dt/6)*(k1+2*k2+2*k3+k4);
    
    % Se guardan las trayectorias del estado y las entradas
    XI(:, k+1) = xi;
    U(:, k+1) = u;
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

trajplot = plot(x, y, '--k', 'LineWidth', 1);

BV = [-0.1, 0, 0.1; 0, 0.3, 0];
IV = [cos(theta-pi/2), -sin(theta-pi/2); sin(theta-pi/2), cos(theta-pi/2)] * BV;
bodyplot = fill(IV(1,:) + x, IV(2,:) + y, [0.5,0.5,0.5]);

xlabel('$x$', 'Interpreter', 'latex', 'Fontsize', 16);
ylabel('$y$', 'Interpreter', 'latex', 'Fontsize', 16);
hold off;

for n = 2:K+1
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