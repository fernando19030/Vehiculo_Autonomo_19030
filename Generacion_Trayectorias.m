%% Trayectoria Cuadrada

cuadrado = [0, 0; 
            1, 0; 
            1, 1; 
            0, 1; 
            0, 0];

bajox = linspace(190,250,50);
bajoy = linspace(240,240,50);

derechax = linspace(250,250,50);
derechay = linspace(240,300,50);

arribax = linspace(250,190,50);
arribay = linspace(300,300,50);

izquierdax = linspace(190,190,50);
izquierday = linspace(300,240,50);

cuadrado_totalx = [bajox, derechax, arribax, izquierdax];
cuadrado_totaly = [bajoy, derechay, arribay, izquierday];

cuadrado_interpolado = zeros(200, 2);

for i = 1:length(cuadrado_totalx)
    cuadrado_interpolado(i, 1) = cuadrado_totalx(i); 
    cuadrado_interpolado(i, 2) = cuadrado_totaly(i); 
    
end

p = cuadrado_interpolado;
save('Trayectoria_Cuadrada', 'p');

figure(1);
plot(cuadrado_interpolado(:, 1), cuadrado_interpolado(:, 2));
title('Trayectoria Cuadrada');
xlabel('X');
ylabel('Y');
ylim([0 480]);
xlim([0 380]);
hold on;

%% Circular
theta = 0:0.01:2*pi; %Rango de theta
r = 35; 
% Coordenadas del circulo
xCenter = 380/2;
yCenter = 480/2;

xCoord = xCenter + r*cos(theta);
yCoord = yCenter + r*sin(theta);

circulo_interpolado = zeros(length(xCoord), 2);

for i = 1:length(xCoord)
    circulo_interpolado(i, 1) = xCoord(i); 
    circulo_interpolado(i, 2) = yCoord(i); 
    
end

p = circulo_interpolado;

save('Trayectoria_Circular', 'p');

figure(2);
plot(xCoord, yCoord);
title('Trayectoria Cuadrada');
xlabel('X');
ylabel('Y');
ylim([0 480]);
xlim([0 380]);
hold on;
