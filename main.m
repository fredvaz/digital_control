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

global A1 A2 a1 a2 b1 b2 g p km hmax vmax v2    

%%                          System Plant Parameters 

% Cross-sectional areas of the tanks [m2]
A1 = pi * 1^2;
A2 = pi * 2^2;

% Cross-sectional areas of the output orifices of the tanks [m2]
a1 = pi * 0.15^2; 
a2 = pi * 0.2^2;

% Cross-sectional areas of the input pipes to the tanks [m2]
b1 = pi * 0.15^2;
b2 = pi * 0.2^2;

% Gravitational constant [m/s2]
g = 9.81;

% Constant
p = b1/(b1 + b2); 

% Motor constant
km = 2.5;

% Maximum height of the tanks [m]
hmax = 10;

% Maximum voltage of the motors
%vmax = 10; % VER MELHOR
%v2 = Vmax*0.5;


%%                          Initial conditions

% Tanks Liquid Level [m]
H1_0 = 1;
H2_0 = 1; 

% Tanks Liquid Level Disturbances [m]
H1_d = 0;
H2_d = 0;

% Liquid Flux Disturbances
f1 = 0;
f2 = 0;

%%                         Fuzzy Control Options 











