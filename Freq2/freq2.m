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
% November 2018; Last revision: 09-November-2018
%--------------------------------------------------------------------------
%
% Description:
%
%--------------------------------------------------------------------------
%
% References:
%
%--------------------------------------------------------------------------

%%                  2ª Frequência 8 de janeiro 2018

%% Problema 1

%sistema
num=[0 0 2]; den=[1 1 1];

% SLIT em espaço de estados
[a,b,c,d] = tf2ss(num,den)

% %discretização introduzindo atraso de transporte
% [phi,gama,C,D]=c2dt(a,b,c,h,tau);
% 
% %obtenção da função de transferência do SLIT %discreto
% [numz,denz]=ss2tf(phi,gama,C,D,1);
% sysd=tf(numz,denz,h)


%% Problema 2

% Considere um processo linear continuo com modelo de estado definido pelas
% seguintes matrizes 
A = [ 0 2; 0.5 -1 ]; 
B = [ 0; 1] ; 
C = [ 1 1 ]; 
D = 0;

% Periodo de Amostragem [s]
h = 0.5;

% O Modelo de Estado Discreto usando o Matlab:
[ phi, gama ] = c2d(A, B, h);


%% a) Pólos através da matriz da dinâmica A (contínuo)
syms s t

% Matriz Identidade
I = eye(size(A));

% Matriz de transicao de estados
phi_s = inv(s*I - A)
phi_t = ilaplace(sym(phi_s), s, t);

% A função de transferência
Gs = C*phi_s*B






