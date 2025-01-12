% Import the Robotics Toolbox
import robotics.*;

% Define the robot's DH parameters based on the provided table
L(1) = Revolute('d', 0, 'a', 0,     'alpha', pi);      % Joint virtual
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

q = [0, 0, 0, -90, 0, -90, 0]; % Define configuração inicial do robô
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

% Target points and orientation
hold on;
plot3(P0(1), P0(2), P0(3), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
plot3(P1(1), P1(2), P1(3), 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'Color', "blue");
plot3(P2(1), P2(2), P2(3), 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'Color', "green");
plot3(P3(1), P3(2), P3(3), 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'Color', "yellow");
plot3(P4(1), P4(2), P4(3), 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'Color', "black");
legend('P0', 'P1', 'P2', 'P3', 'P4');
hold off;

 % Matriz de rotação constante
Rd = [0  0  1;
     1  0  0;
     0 -1  0];

% Combinar posições e orientação em transformações homogêneas
T1 = [Rd, P1; 0 0 0 1];

i = 0;

% Resolver a cinemática inversa
q_solucao = robot.ikine(T1, q, [0, 1, 1, 1 ,1 ,1, 1]); % Ajuste as restrições conforme necessário
disp('Ângulos das juntas (cinemática inversa):');
disp(q_solucao);

% Controle

thetad = q_solucao; % Define configuração desejada
T = robot.fkine(thetad) % Pega pose desejada do efetuador 
pd = transl(T); % Pega vetor de translação do efetuador
Rd = SO3(T); % Pega o objeto SO3 correspondente à rotação do efetuador
Rd = Rd.R(); %Pega matriz de rotação do efetuador

K = 1; % Define ganho
epsilon = 10e-5; % Define critério de parada
e_ant = 1;
e = 0; 
control_sig = zeros(7, 1000); % 7 joints, assume up to 1000 iterations

i = 0;
theta = [0 0 0 -90 0 -90 0]'; % Define configuração inicial do robô

% Control loop visualization
figure(2);
robot.plot(theta');
hold on;
T.plot('rgb');
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
    J = robot.jacob0(theta); % Geometric Jacobian
    T = robot.fkine(theta); % Forward kinematics for current pose
    
    % Compute position and orientation errors
    p = transl(T); % Current translation
    R = SO3(T).R; % Current rotation
    p_err = pd - p; % Translation error
    
    % Compute rotational error using axis-angle
    nphi = rotm2axang2(Rd * R'); 
    nphi_err = nphi(1:3) * nphi(4); % Rotation error (n*phi)
    
    e_ant = e;
    e = [p_err'; nphi_err']; % Combined error vector
    
    % Null space control (to handle redundancy)
    J_pinv = pinv(J); % Pseudo-inverse of Jacobian
    u_null = lambda * (eye(size(J, 2)) - J_pinv * J) * rand(size(theta)); % Redundancy resolution term
    
    % Control law
    u = J_pinv * (K * e) + u_null; % Add null space term
    
    % Update joint configuration
    theta = theta + 0.1 * u;
    
    % Visualization
    robot.plot(theta');
    control_sig(:, i) = u; % Control signal
    err(i) = norm(e); % Error norm
end

% Open a new figure for plotting error norm
figure('Name', 'Error Norm', 'NumberTitle', 'off'); % Opens a new, named window
plot(err(1:i), 'LineWidth', 2);
xlabel('Iterations');
ylabel('Norm of Error: |e|');
title('Error Norm During Control Loop');
grid on;
