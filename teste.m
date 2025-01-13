K = 1; % Define ganho
epsilon = 10e-3; % Define critério de parada
e_ant = 1;
e = 0; 
control_sig = zeros(7, 10000); % 7 joints, assume up to 1000 iterations
joint_angles = zeros(length(q), 10000); % 1000 é o número máximo de iterações

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
j = 0;
deltat = 0.1;
tempo_total = 5;

% Control loop
for t = 0:deltat:tempo_total 
    fprintf('Tempo atual: %.2f segundos\n', t);
    tn = t/tempo_total;
    x = P4(1) + tn * (P1(1) - P4(1)); % x(t) = 1000 (fixo)
    y = P4(2) + tn * (P1(2) - P4(2)); % y (t) = 600 - 1200 * t
    
    pd = [x y 1].';
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
    u_reduced = pinv(J_reduced) * (K * e + [0 -1.2/tempo_total 0 0 0 0].'); % Movimento das juntas 2 a 7

    % Atualiza apenas as juntas 2 a 7
    theta(2:end) = theta(2:end) + 0.1 * u_reduced;

    % Junta 1 permanece fixa
    theta(1) = 0;

    % Visualização e armazenamento de dados
    robot.plot(theta'); 
    plot3(p(1), p(2), p(3), 'r.', 'MarkerSize', 15);
    control_sig(:, j) = [0; u_reduced]; % Adiciona 0 como movimento da junta 1
    err(i) = norm(e);                   % Armazena a norma do erro
    joint_angles(:, j) = theta; % Armazena os ângulos das juntas para cada iteração
    q_seq(:, i) = theta(2:7);
    % Extraia a posição atual do efetuador usando cinemática direta
    pause(deltat)
end

% Open a new figure for plotting control signals
figure('Name', 'Control Signals', 'NumberTitle', 'off'); % Opens a new, named window

% Trim unused columns from control_sig (up to the current iteration)
control_sig_trimmed = control_sig(:, 1:j); 

% Plot control signals for each joint
hold on;
for m = 1:size(control_sig_trimmed, 1) % Loop over all joints
    plot(control_sig_trimmed(m, :), 'DisplayName', ['Joint ', num2str(m)]);
end
hold off;

% Add labels, title, and legend
xlabel('Iterations');
ylabel('Control Signal: u (rad/s)');
title('Control Signals for Each Joint Over Iterations from P3 to P4');
legend('show'); % Display joint labels in the legend
grid on;


% Remover colunas não usadas
joint_angles_trimmed = joint_angles(:, 1:j);

% Abrir uma nova figura para os ângulos das juntas
figure('Name', 'Joint Angles', 'NumberTitle', 'off'); % Abre uma nova janela

% Plotar os ângulos para cada junta
hold on;
for m = 1:size(joint_angles_trimmed, 1) % Loop sobre todas as juntas
    plot(joint_angles_trimmed(m, :), 'DisplayName', ['Joint ', num2str(m)]);
end
hold off;

% Adicionar rótulos, título e legenda
xlabel('Iterations');
ylabel('Joint Angles (rad)');
title('Joint Angles Over Iterations from P3 to P4');
legend('show'); % Exibe a legenda
grid on;
