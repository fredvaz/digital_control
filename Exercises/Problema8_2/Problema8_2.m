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

% Considere um processo linear contínuo com modelo de estado definido pelas
% seguintes matrizes 
A = [ 0 1; 0 -3 ]; 
B = [ 0; 2] ; 
C = [ 1 0 ]; 
D = 0;

% Período de Amostragem [s]
h = 0.1;

% a) e b) O Modelo de Estado Discreto usando o Matlab:
[ phi, gama ] = c2d(A, B, h);


%% a) Espaço de estados discreto do processo
syms s t

% Matriz Identidade
I = eye(size(A));

% Matriz de transição de estados
phi_s = inv(s*I - A);
phi_t = ilaplace(sym(phi_s), s, t);

% Modelo de estado do sistema equivalente discreto
phi_t_h = eval(subs(phi_t, t, h))
gama_h = eval(int(phi_t, t, 0, h))*B


%% b) Os zeros do sistema discreto equivalente
syms z

% 1º Constuir a função de transferência equivalente:
Gz = C * inv( z*I - phi ) * gama;

% Usando do funções do Matlab:
% Função de trasnferência em malha fechada
[ num, den ] = ss2tf(phi, gama, C, D);
Gz = tf(num,den,h)
zeros = zero(Gz)


%% c) P/ ? = 9.0 e ?n = 1 rad/s. calcule o vector dos ganhos do controlador
% por realimentação linear das variáveis de estado

zeta = 9.0;                 % Factor de amortecimento
wn = 1;                     % Frequencia natural não amortecida
wd = wn * sqrt(1 - zeta^2); % Frequencia natural amortecida

% A partir da eq. característica s^2 + 2??d + wd e z = e^sh
% O polinómio Caraterístico vêm que p(z) = z^2 + p1*z + p2
p1 = -2*exp( -zeta*wn*h )*cos( wd*h ); % polo 1
p2 = exp( -2*zeta*wn*h ); % polo 2

% Matriz de controlabilidade
Wc = [ gama phi * gama ];

% Ganhos de realimentação das varíaveis de estado
% obtidos pela fórmula de Ackermann

L = [0 1] * inv(Wc) * (phi^2 + p1 * phi + p2 * eye(2))

% Matriz dinâmica de regulação
phic = phi - gama * L;

% Ganho de avanço Lc
Lc = 1/( C * inv(eye(2) - phic ) * gama )


%% d) Calcule os zeros do sistema de regulação e discuta este resultado face ao resultado
% obtido na alínea b)


















