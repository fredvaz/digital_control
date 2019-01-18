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

% Considere um processo linear continuo com modelo de estado definido pelas
% seguintes matrizes 
A = [ -3 0; 1 0 ]; 
B = [ 1; 0 ] ; 
C = [ 0 1 ]; 
D = 0;

% Periodo de Amostragem [s]
h = 0.5;

% a) e b) O Modelo de Estado Discreto usando o Matlab:
[ phi, gama ] = c2d(A, B, h);

I = eye(size(A));

ganho_dc = 1; % ganho DC unitÃ¡rio


% matrizes usadas no bloco ?State-Space model? no modelo Simulink
% para simular o caso de todas as variáveis de estado acessíveis
Ca = [1 0; 0 1]; Da = [0; 0];


%% Problema 2

%% a) Projecto do Ganho do observador

% Matriz de observabilidade (ligeiramente diferente do preditor)
Woc = [ C*phi; C*phi^2 ];

% Coeficientes do polinÃ³mio caracterÃ­stico do observador [(z-0.2)^2]
% como pedido no enuciado
syms z
eq = expand( (z-0.1)^2 );
po = [ 1 -0.2 0.01 ]; 

% Vector de ganhos do observador 
Ko = (po(1)*phi^2 + po(2)*phi + po(3)*eye(2))*inv(Woc)*[0 1]'

% Matriz de observação
phi_obs = (I - Ko*C)*phi;


%% b) Equação do observador

%% c) Matriz CL-Obs e Pólos do controlador-observador 

% Matriz dinamica de regulacao-observador 
phi_cl_obs = phi_obs - gama * L;

% Pólos
polos_cl_obs = eig(phi_obs);


%% Problema 2

%% a) Pólos e Zeros
syms s t

% Matriz de transição de estados
phi_s = inv(s*I - A);
phi_t = ilaplace(sym(phi_s), s, t);

% A função de transferência
Gs = C*phi_s*B;

% Polinómio caracteristico
eq2 = expand( s*(s + 3) );

% Confirmação
[ num, den ] = ss2tf(A, B, C, D);
Gs = tf(num, den);

% Pólos e Zeros
zeros = zero(Gs);
polos = pole(Gs);

%% b) 


%% c) Vector dos ganhos do controlador de realimentação
% por realimentacaoo linear das variaveis de estado

zeta = 1.5;                 % Factor de amortecimento
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

L = [0 1] * inv(Wc) * (phi^2 + p1 * phi + p2 * eye(2));

% Matriz dinamica de regulacao
phi_cl = phi - gama * L;
% Valores proprios de Phi_cl (polos malha fechada)
polos_mf = eig(phi_cl);


% Ganho de avanco Lc
Lc = ganho_dc/( C*inv( eye(2) - phi_cl )*gama );


%% d)

C2 = [ 0 1 ];
C1 = [ 1 0 ];

% Valor inicial do estado estimado
xobs = [0.2 0.2]'; 

syms z

% Polinómio caracteristico
eq3 = eval( det(z*I - phi_cl) )








