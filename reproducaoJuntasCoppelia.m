% preambulo
clc;

% carrega as bibliotecas de apoio
addpath('./lib/libCoppelia/');

%% Parametros

%%%% Posicao inicial em graus no espaco de juntas
q_i = [0    0 -90    0  0 0].';

% delta t do controlador
deltaT = 0.01;  

%% Configuracoes iniciais

% cria o objeto de comando do comau no coppeliasim
comau = ComauSim();

% inicia a simulacao
comau.startSimulation();

% Define a posicao atual do sm6 como o vetor inicial
q = deg2rad(q_i);

% envia o robo para a posicao inicial
comau.setJointTargetPosition(q);
pause(2);
%q_seq = [45 0 -90 0 -90 0].';
%q_seq = theta(2:7);
% q_seq =  [-0.5061 0.3033 -1.4529 -0.7540 -0.6942 0.8025].';

%% Loop de envio das posicoes de juntas ao robo
total_points = size(q_seq,2);
for i=1:total_points

    fprintf('Enviando ponto: %d/%d \n', i, total_points);

    % envia target position para as juntas do sm6
     % comau.setJointTargetPosition(deg2rad(q_seq(:,i)));
    comau.setJointTargetPosition(q_seq(:,i));

    % pequeno pause de acordo com o deltaT estipulado
     pause(deltaT);
end
pause(5);

% finaliza a simulacao
comau.stopSimulation();





