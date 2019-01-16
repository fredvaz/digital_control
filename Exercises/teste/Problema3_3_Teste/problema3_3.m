clear all;
close all;
clc


%% Problema 3.3 do Livro Exerc�cios Resolvidos (resolu��o com Simulink)

% Pretende-se controlar um processo cont�nuo com fun��o de transfer�ncia G(s)
% de modo a que o sistema completo em malha fechada exiba um comportamento
% t�pico de 2a ordem com factor de amortecimento ? = 0.7 e frequ�ncia natural 
% n�o amortecida wn = 5rad/s.


% Um comportamento de 2� ordem significa que os sistema deve ter 2 P�los


% Processo cont�nuo com fun��o de transfer�ncia G(s)
s = tf('s');
G = 1 / ( s + 1)
[ numG, denG ] = tfdata(G, 'v');


%% a) Controlador Anal�gico PI - Gc(s)

xi = 0.7;                       % Factor de Amortecimento
wn = 5;                         % Frequ�ncia natural n�o amortecida 

Kp_analog = 2 * xi * wn - 1;    % Te�ria de Controlo Cont�nuo - Pela equa��o Caracter�stica
Ki_analog = wn^2; 

Gc = Kp_analog + Ki_analog/s;   % De outra forma (Kp_analog*s + Ki_analog)/s


%% b) Controlador Discreto PI - Gc(z) - Caso pr�tico da aplica��o num MCU
%     Practical rule: 0.2 <= wn*h <= 0.6

h = 0.1;                        % Per�odo de amostragem: h = 0.1 e h = 0.2 dado no enuciado
                                % Quanto menor, melhor a aproxima��o
Kp_discrete = Kp_analog;
Ki_discrete = Ki_analog * h/2;  % Gc(s) = kp + Ki * h/2 * ( (z+1)/(z-1) ) dado no enuciado
                                % Multiplica no Simulink por uma Discrete
                                % Transfer Func -> (z+1)/(z-1)
                                % (Ki/s->ki*(z+1)/(z-1)) passagem p/ o Discreto

                                % Zero-Order Hold, Sample and Hold faz a
                                % Amostragem para tempo Discreto 
%% Simula�ao

t_d = 1;                        % Escal�o 
t_P = 3;                        % Tempo de in�cio da Pertuba��o
P = 0.5;                        % Amplitude da pertuba��o

sim('PI.slx');
figure; hold on; grid on;
plot(u);
plot(p)
plot(y_analog);
plot(y_discrete);
xlabel('t'); legend('u(t)', 'p(t)', 'y(t) - Anal�gico', 'y(t) - Discreto');


%% c) Projecte analiticamente o controlador PI discreto
%     Resolu��o no Livro
























      
      