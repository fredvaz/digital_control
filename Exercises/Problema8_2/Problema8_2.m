clear all
close all
clc

format short 

%--------------------------------------------------------------------------
%
%                             Digital Control
%
%--------------------------------------------------------------------------
% Author: Frederico Vaz
% email: fredvaz8@gmail.com
% November 2018; Last revision:
%--------------------------------------------------------------------------
%
% Description: Controlo com Observador de Estado
%
%--------------------------------------------------------------------------

%% Problema 8.2

% Considere um processo linear continuo com modelo de estado definido pelas
% seguintes matrizes 
A = [ 0 1; 0 -3 ]; 
B = [ 0; 2] ; 
C = [ 1 0 ]; 
D = 0;

% Periodo de Amostragem [s]
h = 0.1;

% a) e b) O Modelo de Estado Discreto usando o Matlab:
[ phi, gama ] = c2d(A, B, h);


%% a) Espaco de estados discreto do processo
syms s t

% Matriz Identidade
I = eye(size(A));

% Matriz de transicao de estados
phi_s = inv(s*I - A);
phi_t = ilaplace(sym(phi_s), s, t);

% Modelo de estado do sistema equivalente discreto
phi_t_h = eval(subs(phi_t, t, h))
gama_h = eval(int(phi_t, t, 0, h))*B


%% b) Os zeros do sistema discreto equivalente
syms z

% 1o Constuir a funcao de transferencia equivalente:
Gz = C * inv( z*I - phi ) * gama;

% Usando do funcoes do Matlab:
% Funcao de trasnferencia em malha fechada
[ num, den ] = ss2tf(phi, gama, C, D);
Gz = tf(num, den, h)
zeros = zero(Gz)


%% c) P/ zeta = 0.9 e Wn = 1 rad/s. calcule o vector dos ganhos do controlador
% por realimentacaoo linear das variaveis de estado

zeta = 0.9;                 % Factor de amortecimento
wn = 1;                     % Frequencia natural nao amortecida
wd = wn * sqrt(1 - zeta^2); % Frequencia natural amortecida

% A partir da eq. caracteristica s^2 + 2??d + wd e z = e^sh
% O polinomio Carateristico vem que p(z) = z^2 + p1*z + p2
p1 = -2*exp( -zeta*wn*h )*cos( wd*h ); % polo 1
p2 = exp( -2*zeta*wn*h );              % polo 2

% Matriz de controlabilidade
Wc = [ gama phi * gama ];

% Ganhos de realimentacao das variaveis de estado
% obtidos pela formula de Ackermann

L = [0 1] * inv(Wc) * (phi^2 + p1 * phi + p2 * eye(2))

% Matriz dinamica de regulacao
phi_cl = phi - gama * L;
% Valores proprios de Phi_cl (polos malha fechada)
polos_mf = eig(phi_cl);


%% d) Calcule os zeros do sistema de regulacao e discuta este resultado face ao resultado
% obtido na alinea b)
syms z

% Funcao de transferencia do sistema de regulacao
Hz_cl = C * inv(z*eye(2) - phi_cl ) * gama

% Usando do funcoes do Matlab:
% Funcao de trasnferencia em malha fechada
[ num, den ] = ss2tf(phi_cl, gama, C, D);
Hz_cl = tf(num,den,h)
zeros = zero(Hz_cl)


%% e) Esquema Simulink

% Referencia
t_ref = 10;

% Pertubcao na carga 
p_load = 0.2; 
t_pert = 20;

% Estado inicial
x0 = [ 1; 1];

% i) Os valores das matrizes A, B, C e D do bloco “Discrete State-Space” 
% [numGzoh, denGzoh] = tfdata(Gz, 'v');
% [ A_, B_, C_, D_ ] = tf2ss(numGzoh, denGzoh); % não dá igual

% Alterou-se o vector C para ter acesso a todas a variaveis
Ca = [ 1 0; 0 1 ];
Da = [ 0; 0 ];

% Acesso as variaveis de estado x1(t) e x2(t)
C1 = [ 1 0 ];
C2 = [ 0 1 ];

% ii) Ganho de avanco Lc
Ganho_DC = 1; % 0.5; % 1º sistema tipo 1

Lc = Ganho_DC/( C * inv(eye(2) - phi_cl ) * gama )

% Se Lc = l1, ganho unitário

% Chamada do modelo Simulink
sim('SimulinkP8_2')
plot(u.time, u.signals.values,'r-', x1.time, x1.signals.values,'b--',...
     x2.time, x2.signals.values,'k:','LineWidth',2)
 
title('Estado do sistema e sinal de comando com perturbaçao')
grid on


%% ------------------ Observador preditor deadbeat ------------------
% de estado aumentado: estados da planta (x1 e x2) e estado da pertubação (phi_a^3)
% e) iii)

global xobs Ko C_a phi_a gama_a;

% Ganhos de realimentacao
Lw = 1;
La = [ L Lw ];

% Valor inicial do estado observado
xobs = [0 0 0]';

% Perturbacao na carga
p_load = 0.5;

% Parâmetros do observador(phi_a, C_a, gama_a p/ 3 estados)
% Se 2 estados, usar phi, C, gama... phi_w é sempre 1
phi_xw = gama; phi_w = 1;
phi_a = phi;   %[ phi phi_xw; 0 0 phi_w ];
C_a = C;       %[C 0];
gama_a = gama; %[ gama; 0];

% Matriz de observabilidade: 3 estados (3 linhas)
Wo = [ C_a; C_a*phi_a ]; % C_a*phi_a^2 ];
% Se inv(Wo) existe, o systema é observável
% Observabilidade=det(Wo)

% Vector de ganhos do observador deadbeat: 3 estados, phi_a^3 | 2 estados, phi^2
% assim como o vector [ 0 1 ]' p/ 2 estados, e phi^2
Ko = phi^2 * inv(Wo) * [ 0 1 ]' % [ 0 0 1 ]'

%Matriz dinâmica observador preditor - ver pag188
%|zI - phi +K*C|=0 
%phi-Ko*C

%% Chamada do modelo Simulink p/ 3 estados
sim('SimulinkP8_2_Preditor') 
plot(y.time, y.signals.values,'b--', u.time, u.signals.values,'r-',...
     p_est.time, p_est.signals.values,'k:','LineWidth',2);
 
title('Saida y(t); comando com perturbaçao u(t)+p(t); perturbaçao estimada')
grid on


%% ------------------ Observador preditor nao deadbeat ------------------
% de estado aumentado de dinâmica dominante de um sistema de 2ª ordem
% estados da planta e estado da perturbacao
% e) iv)

alfa0 = 2; alfa1 = 4;
% Sistema de 2ª ordem: eq. caracteristica
den = conv([1 2*zeta*wn*alfa0 (wn*alfa0)^2], [1 alfa1]);

% Calcula funcao de transferiencua p/ dada eq. caracteristica (em Continuo)
[ a, b, c, d ] = tf2ss([0 0 0 1], den);
[ phio, gamao ]= c2d(a, b, h);
% Pólos em Discreto
po = eig(phio);

% Vector de ganhos do observador deadbeat: Formula de Ackerman
% L = (po(1)*phi^2 + po(2)*phi + po(3)*eye(2))*inv(Woc)*[0 1]'
Ko = acker(phi_a', C_a', po)
Ko = Ko';


% Chamada do modelo Simulink
sim('SimulinkP8_2_Preditor') 
plot(y.time, y.signals.values,'b--', u.time, u.signals.values,'r-',...
     p_est.time, p_est.signals.values,'k:','LineWidth',2);
 
title('Saida y(t); comando com perturbaçao u(t)+p(t); perturbaçao estimada')
grid on













































