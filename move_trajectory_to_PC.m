K = 1; % Define ganho
epsilon = 10e-3; % Define critério de parada
e_ant = 1;
e = 0; 

T1 = robot.fkine(q_solucao);

% % Control loop visualization
figure(2);
robot.plot(theta');
hold on;
%T1.plot('rgb');
title('Robot Path During Control Loop');
xlabel('X-axis (m)');
ylabel('Y-axis (m)');
zlabel('Z-axis (m)');
grid on;
view(3);

% Redundancy resolution factor (null space control)
j_ant = j + 1;
n_iteracoes = 300; % Número total de iterações desejadas
tempo_total = 30;
deltat = tempo_total / n_iteracoes; %
PC =  [1; 0; 0.3];
% Control loop
tic;
for ts = 1:n_iteracoes
    iter_start = tic;
    t = (ts - 1) * deltat; % Tempo atual
    %fprintf('Tempo atual: %.2f segundos\n', t);
    x = P2(1) + t * (PC(1) - P2(1))/tempo_total; 
    y = P2(2) + t * (PC(2) - P2(2))/tempo_total; 
    z = P2(3) + t * (PC(3) - P2(3))/tempo_total; 
    
    pd = [x y z].';
    i = i + 1; % Counter
    j = j + 1; % Counter

    % Calcula a jacobiana completa e cinemática direta
    J_full = robot.jacob0(theta); % Jacobiana completa (6x7)
    T = robot.fkine(theta);       % Pose atual do efetuador

    % Reduz a jacobiana (remove a contribuição da junta 1)
    J_reduced = J_full(:, 2:end); % Jacobiana reduzida (6x6)

    % Erros de posição e orientação
    p = transl(T);           % Extração da posição
    R = SO3(T).R;            % Extração da rotação
    p_err = pd' - p;          % Erro de posição
    nphi = rotm2axang2(Rd * R'); % Erro de rotação (em eixo-ângulo)
    nphi_err = nphi(1:3) * nphi(4); % Vetor n * phi (parte do erro angular)

    % Vetor de erro combinado
    e_ant = e; % Atualizar erro anterior
    e = [p_err'; nphi_err'];

    % Resolve o controle com a jacobiana reduzida
    u_reduced = pinv(J_reduced) * (K * e + [0 0.6/tempo_total 0 0 0 0].'); % Movimento das juntas 2 a 7

    % Atualiza apenas as juntas 2 a 7
    theta(2:end) = theta(2:end) + u_reduced;

    % Junta 1 permanece fixa
    theta(1) = 0;

    % Visualização e armazenamento de dados
    robot.plot(theta'); 
    plot3(p(1), p(2), p(3), 'r.', 'MarkerSize', 3);
    control_sig(:, j) = [0; u_reduced]; % Adiciona 0 como movimento da junta 1
    err(j) = norm(e);                   % Armazena a norma do erro
    joint_angles(:, j) = theta; % Armazena os ângulos das juntas para cada iteração
    q_seq(:, j) = theta(2:7);
    err_rot(:, j) =  [nphi_err(1) nphi_err(2) nphi_err(3)];
    % Extraia a posição atual do efetuador usando cinemática direta
    
    elapsed = toc(iter_start); % Tempo gasto na iteração
    pause_time = deltat - elapsed; % Tempo restante para o próximo passo
    if pause_time > 0
        pause(pause_time); % Pausa ajustada
    end
end
toc;
% Open a new figure for plotting control signals
figure('Name', 'Control Signals', 'NumberTitle', 'off'); % Opens a new, named window

% Trim unused columns from control_sig (up to the current iteration)
control_sig_trimmed = control_sig(:, j_ant:j); 

% Plot control signals for each joint
hold on;
for m = 1:size(control_sig_trimmed, 1) % Loop over all joints
    plot(control_sig_trimmed(m, :), 'DisplayName', ['Junta ', num2str(m)]);
end
hold off;


% Add labels, title, and legend
xlabel('Iteraçoes');
ylabel('Sinal de Controle: u (rad/s)');
title('Sinal de Controle para cada Junta de P2 a PC');
legend('show'); % Display joint labels in the legend
grid on;



% Remover colunas não usadas
joint_angles_trimmed = joint_angles(:, j_ant:j);

% Abrir uma nova figura para os ângulos das juntas
figure('Name', 'Joint Angles', 'NumberTitle', 'off'); % Abre uma nova janela

% Plotar os ângulos para cada junta
hold on;
for m = 1:size(joint_angles_trimmed, 1) % Loop sobre todas as juntas
    plot(joint_angles_trimmed(m, :), 'DisplayName', ['Junta ', num2str(m)]);
end
hold off;


% Adicionar rótulos, título e legenda
xlabel('Iteraçoes');
ylabel('Angulo das Juntas(rad)');
title('Angulos das Juntas de P2 a PC');
legend('show'); % Exibe a legenda
grid on;


% Abrir uma nova figura para os ângulos das juntas
figure('Name', 'Error Norm from  P2 to PC', 'NumberTitle', 'off'); % Abre uma nova janela

% Plotar os ângulos para cada junta
plot(err(j_ant:j), 'LineWidth', 1.5);
xlabel('Tempo (s)');
ylabel('Erro de Posição (mm)');
title('Erro de Posição de P2 a PC');
grid on;


% Remover colunas não usadas
err_rot_trimmed = err_rot(:, j_ant:j);

% Abrir uma nova figura para os ângulos das juntas
figure('Name', 'Error Row Pitch Yaw from P1 to P2', 'NumberTitle', 'off'); % Abre uma nova janela

% Plotar os ângulos para cada junta
hold on;
for n = 1:size(err_rot_trimmed, 1) % Loop sobre todas as juntas
    plot(err_rot_trimmed(n, :), 'DisplayName', ['Row', 'Pitch', 'Yaw']);
end
hold off;

% Adicionar rótulos, título e legenda
xlabel('Tempo (s)');
ylabel('Erro de Orientação (graus)');
title('Erro de Orientação de P2 a PC');
legend('Roll', 'Pitch', 'Yaw');
grid on;

