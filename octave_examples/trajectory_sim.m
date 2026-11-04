
waypoints = [
    35.78, 35.78, 10.0;  % Point de départ
    20.00, 30.00, 20.0;  % Point de prise
    -25.00, 20.00, 15.0; % Point de dépose
    0.00, 0.00, 30.0     % Retour
];

% 2. Tracé de la trajectoire
figure;
hold on;
grid on;
view(3);

% Dessiner les lignes entre les points
plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), '-o', 'LineWidth', 2, 'Color', 'b', 'MarkerFaceColor', 'r');

% Ajouter le sol (Z=0)
[X, Y] = meshgrid(-50:5:50, -50:5:50);
Z = zeros(size(X));
surf(X, Y, Z, 'FaceAlpha', 0.2, 'EdgeColor', 'none');

% Mise en forme du graphique
xlabel('X (cm)');
ylabel('Y (cm)');
zlabel('Z (cm)');
title('Simulation de Trajectoire Robotique (Généré par Octave)');
legend('Trajectoire de la pince', 'Sol (Z=0)');
axis equal;

disp('Simulation terminée. Regarde le graphique !');