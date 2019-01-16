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

global xobs Ko C phi gama;

% Considere um processo linear cont�nuo com modelo de estado definido pelas
% seguintes matrizes 
A = [ -3 0; 1 0 ]; 
B = [ 1; 0] ; 
C = [ 0 1 ]; % sa�da x2
D = 0;

% Per�odo de Amostragem [s]
h = 0.2;

% a) e b) O Modelo de Estado Discreto usando o Matlab:
[ phi, gama ] = c2d(A, B, h);

% Acesso as variav�is de estado interno
C1 = [ 1 0 ];
C2 = [ 0 1 ];

% Estado incial
x0 = [ 0 0 ]';

% Ganho DC
ganho_dc = 1;

% Acesso a todas das variaveis
Ca = [ 1 0; 0 1 ];
Da = [ 0; 0];


%% R1) P/ zeta = 0.8 e Wn = 2 rad/s. calcule o vector dos ganhos do controlador
% por realimenta��o linear das vari�veis de estado

zeta = 0.8;                 % Factor de amortecimento
wn = 2;                     % Frequencia natural n�o amortecida
wd = wn * sqrt(1 - zeta^2); % Frequencia natural amortecida

% A partir da eq. caracter�stica s^2 + 2??d + wd e z = e^sh
% O polin�mio Carater�stico v�m que p(z) = z^2 + p1*z + p2
p1 = -2*exp( -zeta*wn*h )*cos( wd*h ); % polo 1
p2 = exp( -2*zeta*wn*h ); % polo 2

% Matriz de controlabilidade
Wc = [ gama phi * gama ];

% Ganhos de realimenta��o das var�aveis de estado
% obtidos pela f�rmula de Ackermann

L = [0 1] * inv(Wc) * (phi^2 + p1 * phi + p2 * eye(2))

% Matriz din�mica de regula��o
phi_cl = phi - gama * L;
% Valores pr�prios de Phi_cl p�los
polo_mf = eig(phi_cl)

% Ganho de avan�o Lc
Lc = ganho_dc/( C * inv(eye(2) - phi_cl ) * gama )


%% R6) Observador preditor n�o Deadbeat

% estado inicial nulo
xobs = [ 0; 0];

% Polo duplo z=0.22
z = 0.22;            
den = conv([1 z], [1 z]);

% Din�mica de regula��o z1,2 = a +-jb 
% Para p�lo duplo em z = 0.22
a = 0.22; b = 0; 

% Do polin�mio caracter�stico z^2 + p1 x z + p2 v�m que:
p1 = -2 * a; % -0.4400; % - ??
p2 = a^2 + b^2; 

% Matriz de observalidade
Wo = [ C; C*phi ];

% Ganhos de realimenta��o das var�aveis de estado
% obtidos pela f�rmula de Ackermann

Ko = (phi^2 + p1 * phi + p2 * eye(2)) * inv(Wo) * [0 1]' 

% Matriz din�mica de observa��o
phi_o = phi - Ko * C;
% Valores proprios: polos
po = eig(phi_o)


%% Confirma��o

num = [0 0 1];
%den = [1 p1 p2]; % est� mal, isto � continuo, e temos z=0.22 em discreto

Gz = tf(num,den,h)
polos = pole(Gz)


%%
%[a,b,c,d] = tf2ss([0 0 1],den);
%[phio,gamao] = c2d(a,b,h);

po = eig(phio)

Ko = acker(phi',C',po)
Ko = Ko';
















%% d) Calcule os zeros do sistema de regula��o e discuta este resultado face ao resultado
% obtido na al�nea b)



%% Observador preditor Deadbeat

% AQUI n�o fazemos o Modelo do sistema aumentado com modelo da perturbacao

% Matriz de observabilidade
Wo = [ C; C*phi ];

%Vector de ganhos do observador deadbeat
Ko = phi^2 * inv(Wo) * [ 0 1 ]'

%valor inicial do estado observado
xobs = [ 0 0 ]';
%perturbacao na carga
p_load = 0.5;


%% Chamada o modelo Simulink

% Acesso a todas as variaveis, sa�da c/ x1 e x2;
Ca=[1 0;0 1]; Da=[0;0];

sim('Problema8_2') 
plot(y.time, y.signals.values,'b--', u.time, u.signals.values,'r-',...
p_est.time, p_est.signals.values,'k:','LineWidth',2);

title('Saida y(t); comando com perturba�ao u(t)+p(t); perturba�ao estimada')
grid


%% Chamada o modelo Simulink

% Acesso a todas as variaveis, sa�da c/ x1 e x2;
Ca=[1 0;0 1]; Da=[0;0];

sim('Problema8_2_Preditor_Teste') 


%% a) Espa�o de estados discreto do processo
syms s t

% Matriz Identidade
I = eye(size(A));

% Matriz de transi��o de estados
phi_s = inv(s*I - A);
phi_t = ilaplace(sym(phi_s), s, t);

% Modelo de estado do sistema equivalente discreto
phi_t_h = eval(subs(phi_t, t, h))
gama_h = eval(int(phi_t, t, 0, h))*B


%% b) Os zeros do sistema discreto equivalente
syms z

% 1� Constuir a fun��o de transfer�ncia equivalente:
Gz = C * inv( z*I - phi ) * gama;

% Usando do fun��es do Matlab:
% Fun��o de trasnfer�ncia em malha fechada
[ num, den ] = ss2tf(phi, gama, C, D);
Gz = tf(num,den,h)
zeros = zero(Gz)

















