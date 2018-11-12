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

%%                          System Plant Parameters 

global A1 A2 a1 a2 b1 b2 g p km vmax hmax v2    

% Cross-sectional areas of the tanks [m2]
A1 = pi * 1^2;
A2 = pi * 2^2;

% Cross-sectional areas of the output orifices of the tanks [m2]
a1 = pi * 0.15^2; % [m2]
a2 = pi * 0.2^2; % [m2]

% Cross-sectional areas of the input pipes to the tanks [m2]
b1 = pi * 0.15^2;
b2 = pi * 0.2^2;

% Gravitational constant [m/s2]
g = 9.81;

% Constant
p = b1/(b1 + b2);

% Motor constant
km = 2.5;

% Maximum height of the tanks
hmax = 10; 

% Maximum voltage of the motors
vmax = 10;


%%                         Fuzzy Control Options 











