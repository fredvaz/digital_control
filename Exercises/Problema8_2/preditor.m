
%% Observador Preditor
function estim = preditor(ent)

global xobs Ko C_a phi_a gama_a;

uk1 = ent(1);
yok = ent(2);

% Predição do estado aumentado
xobs = phi_a*xobs + gama_a*uk1 + Ko*(yok-C_a*xobs);

% Devolução do estado observado
estim = xobs;

end