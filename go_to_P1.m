% Import the Robotics Toolbox
import robotics.*;

% Define the robot's DH parameters based on the provided table
L(1) = Revolute('d', 0, 'a', 0,     'alpha', -pi);      % Joint virtual
L(2) = Revolute('d', -0.45, 'a', 0.15,     'alpha', pi/2);         % Joint 1
L(3) = Revolute('d', 0,     'a', 0.59, 'alpha', pi, 'offset', -pi/2); % Joint 2
L(4) = Revolute('d', 0,     'a', 0.13, 'alpha', -pi/2, 'offset', pi/2);  % Joint 3
L(5) = Revolute('d', -0.6471, 'a',0, 'alpha', -pi/2);    % Joint 4
L(6) = Revolute('d', 0,     'a', 0,     'alpha', pi/2);      % Joint 5
L(7) = Revolute('d', -0.095, 'a', 0,     'alpha', pi, 'offset', pi);      % Joint 6

% Combine the links into a SerialLink robot
robot = SerialLink(L, 'name', '6-DOF Robot');

% Display the robot's structure
robot.display();

q = [0, 0, 0, -pi/2, 0, -pi/2, 0]; % Define configuração inicial do robô
figure(1)

% Compute the Jacobian in the base frame
J = robot.jacob0(q);

% Display the Jacobian
disp('Jacobian Matrix (Base Frame):');
disp(J);

% Extrair apenas as 6 ultimas colunas (jacobiana reduzida para posição)
J_reduced = J(:, 2:7);

disp('Jacobiana Reduzida (para posição):');
disp(J_reduced);


%% Posições alvo (em mm convertidas para metros)
 P0 = [0.7; 0; 0.7];    % [700; 0; 700]
 P1 = [1; -0.6; 1]; % [1000; -600; 1000]
 P2 = [1; -0.6; 0.3]; % [1000; -600; 300]
 P3 = [1; 0.6; 0.3]; % [1000; 600; 300]
 P4 = [1; 0.6; 1]; % [1000; 600; 1000]

% Initial robot plot with workspace configuration
figure(1);
robot.plotopt = {'workspace', [-2 2 -2 2 -2 2], 'view', [120, 30]};
robot.plot(q);
title('Initial Configuration of the Robot');
xlabel('X-axis (m)');
ylabel('Y-axis (m)');
zlabel('Z-axis (m)');
grid on;

pause(3);
% Target points and orientation
hold on;
plot3(P0(1), P0(2), P0(3), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
plot3(P1(1), P1(2), P1(3), 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'Color', "blue");
plot3(P2(1), P2(2), P2(3), 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'Color', "green");
plot3(P3(1), P3(2), P3(3), 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'Color', "yellow");
plot3(P4(1), P4(2), P4(3), 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'Color', "black");
%legend('P0', 'P1', 'P2', 'P3', 'P4');
hold off;

 % Matriz de rotação constante
    Rd = [0 0 1;
         0  1  0;
         -1 0 0];

i = 0;
pd = P1;

K = 1; % Define ganho
epsilon = 10e-4; % Define critério de parada
e_ant = 1;
e = 0; 

control_sig = zeros(7, 1000); % 7 joints, assume up to 1000 iterations
joint_angles = zeros(length(q), 1000); % 1000 é o número máximo de iterações
historical_pos = zeros(3, 1000); % 1000 é o número máximo de iterações
theta = [0 0 0 -pi/2 0 -pi/2 0]'; % Define configuração inicial do robô
T1 = [Rd, P1; 0 0 0 1];
% Resolver a cinemática inversa
q_solucao = robot.ikine(T1, q, [0, 1, 1, 1 ,1 ,1, 1]); % Ajuste as restrições conforme necessário
disp('Ângulos das juntas (cinemática inversa):');
disp(q_solucao);

T1 = robot.fkine(q_solucao);

% Control loop visualization
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

% Control loop
while (norm(e - e_ant) > epsilon) % Stopping criterion
    i = i + 1; % Counter

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
    hold on;
    plot3(p(1), p(2), p(3), 'b.', 'MarkerSize', 15);
    hold off;
    control_sig(:, i) = [0; u_reduced]; % Adiciona 0 como movimento da junta 1
    err(i) = norm(e);                   % Armazena a norma do erro
    joint_angles(:, i) = theta; % Armazena os ângulos das juntas para cada iteração
    q_seq(:, i) = theta(2:7);
    disp('Ângulos finais das juntas:');
    disp(theta);
end

% Open a new figure for plotting control signals
figure('Name', 'Control Signals', 'NumberTitle', 'off'); % Opens a new, named window

% Trim unused columns from control_sig (up to the current iteration)
control_sig_trimmed = control_sig(:, 1:i); 

% Plot control signals for each joint
hold on;
for j = 1:size(control_sig_trimmed, 1) % Loop over all joints
    plot(control_sig_trimmed(j, :), 'DisplayName', ['Joint ', num2str(j)]);
end
hold off;

% Add labels, title, and legend
xlabel('Iterations');
ylabel('Control Signal: u (rad/s)');
title('Control Signals for Each Joint Over Iterations from P0 to P1');
legend('show'); % Display joint labels in the legend
grid on;


% Remover colunas não usadas
joint_angles_trimmed = joint_angles(:, 1:i);
    
% Abrir uma nova figura para os ângulos das juntas
figure('Name', 'Joint Angles', 'NumberTitle', 'off'); % Abre uma nova janela

% Plotar os ângulos para cada junta
hold on;
for j = 1:size(joint_angles_trimmed, 1) % Loop sobre todas as juntas
    plot(joint_angles_trimmed(j, :), 'DisplayName', ['Joint ', num2str(j)]);
end
hold off;

% Adicionar rótulos, título e legenda
xlabel('Iterations');
ylabel('Joint Angles (rad)');
title('Joint Angles Over Iterations from P0 to P1');
legend('show'); % Exibe a legenda
grid on;