K = 1; % Define ganho
epsilon = 10e-4; % Define critério de parada
e_ant = 1;
e = 0; 

%theta = [0 0 0 -pi/2 0 -pi/2 0]'; % Define configuração inicial do robô
T2 = [Rd, P2; 0 0 0 1];
pd = P2;
% Resolver a cinemática inversa
q_solucao = robot.ikine(T2, theta, [0, 1, 1, 1 ,1 ,1, 1]); % Ajuste as restrições conforme necessário
disp('Ângulos das juntas (cinemática inversa):');
disp(q_solucao);

T1 = robot.fkine(q_solucao);

%Control loop visualization
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
lambda = 0.01; % Tuning parameter for redundancy resolution
j_ant = i + 1;
j  = i;
% Control loop
while (norm(e - e_ant) > epsilon) % Stopping criterion
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
    u_reduced = pinv(J_reduced) * (K * e); % Movimento das juntas 2 a 7

    % Atualiza apenas as juntas 2 a 7
    theta(2:end) = theta(2:end) + 0.1 * u_reduced;

    % Junta 1 permanece fixa
    theta(1) = 0;

    % Visualização e armazenamento de dados
    robot.plot(theta'); 
    plot3(p(1), p(2), p(3), 'b.', 'MarkerSize', 3);
    control_sig(:, j) = [0; u_reduced]; % Adiciona 0 como movimento da junta 1
    err(j) = norm(e);                   % Armazena a norma do erro   
    err_rot(:, j) =  [nphi_err(1) nphi_err(2) nphi_err(3)];
    joint_angles(:, j) = theta; % Armazena os ângulos das juntas para cada iteração
    q_seq(:, j) = theta(2:7);
    disp('Ângulos finais das juntas:');
    disp(theta);
end

% Open a new figure for plotting control signals
figure('Name', 'Control Signals', 'NumberTitle', 'off'); % Opens a new, named window

% Trim unused columns from control_sig (up to the current iteration)
control_sig_trimmed = control_sig(:, j_ant:j); 

% Plot control signals for each joint
hold on;
for m = 1:size(control_sig_trimmed, 1) % Loop over all joints
    plot(control_sig_trimmed(m, :), 'DisplayName', ['Joint ', num2str(m)]);
end
hold off;

% Add labels, title, and legend
xlabel('Iterations');
ylabel('Control Signal: u (rad/s)');
title('Control Signals for Each Joint Over Iterations from P1 to P2');
legend('show'); % Display joint labels in the legend
grid on;


% Remover colunas não usadas
joint_angles_trimmed = joint_angles(:, j_ant:j);

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
title('Joint Angles Over Iterations from P1 to P2');
legend('show'); % Exibe a legenda
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
title('Erro de Orientação');
legend('Roll', 'Pitch', 'Yaw');
grid on;
