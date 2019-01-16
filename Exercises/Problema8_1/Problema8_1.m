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

%% Problema 8.1
global xobs Ko C phi gama

% Considere um sistema discreto com o seguinte modelo de estado: 
Phi12 = 1.5;
%Phi12=-1.5

phi = [1 Phi12; 0 0.4];
gama = [ 2; 1 ]; 
C = [ 1 0 ]; D = 0;

% Coeficientes do polinómio característico em tempo continuo desejado:
% Am(s)= s^2 + a1*s + a2
a1 = 1.8; a2 = 1.25;

Wnh = 0.4;    % Wn*h=0.4 e  
ganho_dc = 1; % ganho DC unitário

% Matrizes usadas no esquema de Simulink, para se obter à saída do
% sistema discreto o vector de estados
Ca = [ 1 0; 0 1 ]; Da = [ 0; 0 ]; 

% Acesso a variavel x2 no modelo de Simulink
C2 = [0 1]; 


%% b) Projecto do Controlador

% Funcao de transferencia do sistema discreto: H=(B/A) MALHA ABERTA
sys = ss(phi, gama, C, D, []);
H = tf(sys)

% Do polinomio caracteristico continuo determina-se
Wn = sqrt(a2); zeta = a1/(2*Wn);

% período de amostragem (h=wn*h)
h = Wnh/Wn; 

% p1 e p2 são os coeficientes do polinómio característico discreto
% desejado Am(z) = z^2 + p1z + p2
p1 = -2 * exp( -zeta*Wnh ) * cos( Wnh*sqrt(1 - zeta^2) );
p2 = exp( -2*zeta*Wnh );

% Matriz de controlabilidade
Wc = [ gama phi*gama ]; 

% Ganhos de realimentação das variáveis de estado
L = [0 1] * inv(Wc)*(phi^2 + p1*phi + p2*eye(2))


% Matriz de transição de estados em malha fechada
phi_cl = phi - gama*L;
% Valores proprios de Phi_cl (polos malha fechada)
polos_mf = eig(phi_cl);

% Ganho de avanco Lc
Lc = ganho_dc/( C*inv( eye(2) - phi_cl )*gama );


% NOVA funcao de transferência discreta em MALHA FECHADA
sys = ss(phi_cl, gama, C, D, []); 
Hc = tf(sys)
% Hcl=Lc*(B/Am)
Hcl= Lc*Hc; 


%% c) Esquema Simulink

% Chamada do modelo Simulink
sim('SimulinkP8_1')
plot(yk.time, yk.signals.values,'k*', x2k.time, x2k.signals.values, 'ko');
xlabel('Instante de amostragem k'); ylabel('amplitude')
grid on


%% d) Repita as alíneas b) e c) para Phi12 = −1.5 mantendo Wnh = 0.4

% Chamada do modelo Simulink
sim('SimulinkP8_1')
plot(yk.time, yk.signals.values,'k*', x2k.time, x2k.signals.values, 'ko');
xlabel('Instante de amostragem k'); ylabel('amplitude')
grid on


%% e) Projecto do Ganho do observador

% Valor inicial do estado estimado
xobs = [0 0]'; 

% Matriz de observabilidade (ligeiramente diferente do preditor)
Woc = [ C*phi; C*phi^2 ];

% Coeficientes do polinómio característico do observador [(z-0.2)^2]
% como pedido no enuciado
po = [ 1 -0.4 0.04 ]; 

% Vector de ganhos do observador 
Ko = (po(1)*phi^2 + po(2)*phi + po(3)*eye(2))*inv(Woc)*[0 1]'


% Chamada do modelo Simulink
sim('SimulinkP8_1_Corrente')
plot(yk.time, yk.signals.values,'k*', x2k_est.time, x2k_est.signals.values, 'ko');
xlabel('Instante de amostragem k'); ylabel('amplitude')
grid on

















