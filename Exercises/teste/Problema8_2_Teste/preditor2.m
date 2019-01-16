function estim = preditor(ent)

global xobs Ko C phi gama;

uk1=ent(1);
yok=ent(2);

%predição do estado aumentado
xobs=phi*xobs+gama*uk1+Ko*(yok-C*xobs)

%Devolução do estado observado
estim=xobs