go_to_P1()
go_to_P2()
go_to_P3()
go_to_P4()
move_trajectory_to_P1()
move_trajectory_to_P2()
move_trajectory_to_PC()
move_trajectory_circle()

% Open a new figure for plotting control signals
figure('Name', 'Control Signals', 'NumberTitle', 'off'); % Opens a new, named window

% Trim unused columns from control_sig (up to the current iteration)
control_sig_trimmed = control_sig(:, 1:j); 

% Plot control signals for each joint
hold on;
for n = 1:size(control_sig_trimmed, 1) % Loop over all joints
    plot(control_sig_trimmed(n, :), 'DisplayName', ['Joint ', num2str(n)]);
end
hold off;

% Add labels, title, and legend
xlabel('Iterations');
ylabel('Control Signal: u (rad/s)');
title('Control Signals for Each Joint Over Iterations');
legend('show'); % Display joint labels in the legend
grid on;


% Remover colunas não usadas
joint_angles_trimmed = joint_angles(:, 1:j);

% Abrir uma nova figura para os ângulos das juntas
figure('Name', 'Joint Angles', 'NumberTitle', 'off'); % Abre uma nova janela

% Plotar os ângulos para cada junta
hold on;
for n = 1:size(joint_angles_trimmed, 1) % Loop sobre todas as juntas
    plot(joint_angles_trimmed(n, :), 'DisplayName', ['Joint ', num2str(n)]);
end
hold off;

% Adicionar rótulos, título e legenda
xlabel('Iterations');
ylabel('Joint Angles (rad)');
title('Joint Angles Over Iterations');
legend('show'); % Exibe a legenda
grid on;

% Abrir uma nova figura para os ângulos das juntas
figure('Name', 'Error Norm', 'NumberTitle', 'off'); % Abre uma nova janela

% Plotar os ângulos para cada junta
plot(err(1:i), 'LineWidth', 1.5);
xlabel('Tempo (s)');
ylabel('Erro de Posição (mm)');
title('Erro de Posição');
grid on;



% Remover colunas não usadas
err_rot_trimmed = err_rot(:, 1:i);

% Abrir uma nova figura para os ângulos das juntas
figure('Name', 'Error Row Pitch Yaw', 'NumberTitle', 'off'); % Abre uma nova janela

% Plotar os ângulos para cada junta
hold on;
for n = 1:size(err_rot_trimmed, 1) % Loop sobre todas as juntas
    plot(err_rot_trimmed(n, :), 'DisplayName', ['Row', 'Pitch', 'Yaw']);
end
hold off;

% Adicionar rótulos, título e legenda
xlabel('Tempo (s)');
ylabel('Erro de Orientação (graus)');
title('Erro de Orientação');
legend('Roll', 'Pitch', 'Yaw');
grid on;
