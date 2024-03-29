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
% Description: Controlo por realimenta��o das vari�veis de estado 
%
%--------------------------------------------------------------------------

%% Problema 7.2

% Pretende-se controlar por computador um sistema cont�nuo que possui o 
% modelo de estado cont�nuo:

A = [  0  1; -2 -3 ]; 
B = [ 0; 4];
C = [ 1 0 ];  % acesso a vari�vel de estado 1
C1 = [ 0 1 ]; % acesso a vari�vel de estado 2
D = 0;

% Per�odo de Amostragem [s]
h = 0.1; % 0.05;

% a) e b) O Modelo de Estado Discreto usando o Matlab:
[ phi, gama ] = c2d(A, B, h);

% matrizes usadas no bloco ?State-Space model? no modelo Simulink
% para simular o caso de todas as vari�veis de estado acess�veis
Ca = [1 0; 0 1]; Da = [0; 0];


%% a) Calcule a matriz de transi��o de estados
syms s t
% Matriz Identidade
I = eye(size(A));

phi_s = inv(s*I - A)
phi_t = ilaplace(sym(phi_s), s, t)


%% b) Calcule o modelo de estado do sistema equivalente discreto

phi_t_h = eval(subs(phi_t, t, h))
gama_h = eval(int(phi_t, t, 0, h))*B


%% c) Calcule os ganhos de realimenta��o das vari�veis de estado, p/

% Os p�los
z1 = 0.4 + 0.3i;
z2 = 0.4;           % P�lo Duplo           

% Din�mica de regula��o z1,2 = a +-jb 

a = 0.4; b = 0.3;     % P�lo z1
% b=0; %Para p�lo duplo em z = 0.4

% Do polin�mio caracter�stico z^2 + p1 x z + p2 v�m que:
p1 = -2 * a; 
p2 = a^2 + b^2; 

% Matriz de controlabilidade
Wc = [ gama phi * gama ];

% Ganhos de realimenta��o das var�aveis de estado
% obtidos pela f�rmula de Ackermann

L = [0 1] * inv(Wc) * (phi^2 + p1 * phi + p2 * eye(2))

% Matriz din�mica de regula��o
phic = phi - gama * L;

% Ganho de avan�o Lc
Lc = 1/( C * inv(eye(2) - phic ) * gama )


%% Simula��o
sim('SimulinkP7_2.slx');

% Curva da Resposta y(t) e y(k) e da vari�vel de estado interno
subplot(2,1,1)
plot(yc.time,yc.signals.values,'k',yk.time,yk.signals.values,'ro-');
xlabel('tempo (s)'); ylabel('amplitude')
grid on
title('Para h = 0.1');

% Curva da Resposta y(t) da vari�vel de estado interno x2(t)
subplot(2,1,2)
plot(yc1.time,yc1.signals.values,'k',x2.time,x2.signals.values,'r');
xlabel('tempo (s)'); ylabel('amplitude')
grid on










































