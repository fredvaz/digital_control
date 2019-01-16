clear all;
close all;
clc


%% Problema 3.3 do Livro Exercícios Resolvidos (resolução com Simulink)

% Pretende-se controlar um processo contínuo com função de transferência G(s)
% de modo a que o sistema completo em malha fechada exiba um comportamento
% típico de 2a ordem com factor de amortecimento ? = 0.7 e frequência natural 
% não amortecida wn = 5rad/s.


% Um comportamento de 2ª ordem significa que os sistema deve ter 2 Pólos


% Processo contínuo com função de transferência G(s)
s = tf('s');
G = 1 / ( s + 1)
[ numG, denG ] = tfdata(G, 'v');


%% a) Controlador Analógico PI - Gc(s)

xi = 0.7;                       % Factor de Amortecimento
wn = 5;                         % Frequência natural não amortecida 

Kp_analog = 2 * xi * wn - 1;    % Teória de Controlo Contínuo - Pela equação Característica
Ki_analog = wn^2; 

Gc = Kp_analog + Ki_analog/s;   % De outra forma (Kp_analog*s + Ki_analog)/s


%% b) Controlador Discreto PI - Gc(z) - Caso prático da aplicação num MCU
%     Practical rule: 0.2 <= wn*h <= 0.6

h = 0.1;                        % Período de amostragem: h = 0.1 e h = 0.2 dado no enuciado
                                % Quanto menor, melhor a aproximação
Kp_discrete = Kp_analog;
Ki_discrete = Ki_analog * h/2;  % Gc(s) = kp + Ki * h/2 * ( (z+1)/(z-1) ) dado no enuciado
                                % Multiplica no Simulink por uma Discrete
                                % Transfer Func -> (z+1)/(z-1)
                                % (Ki/s->ki*(z+1)/(z-1)) passagem p/ o Discreto

                                % Zero-Order Hold, Sample and Hold faz a
                                % Amostragem para tempo Discreto 
%% Simulaçao

t_d = 1;                        % Escalão 
t_P = 3;                        % Tempo de início da Pertubação
P = 0.5;                        % Amplitude da pertubação

sim('PI.slx');
figure; hold on; grid on;
plot(u);
plot(p)
plot(y_analog);
plot(y_discrete);
xlabel('t'); legend('u(t)', 'p(t)', 'y(t) - Analógico', 'y(t) - Discreto');


%% c) Projecte analiticamente o controlador PI discreto
%     Resolução no Livro
























      
      