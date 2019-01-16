
%% Observador Corrente
function output = corrente(input)
    
    global xobs Ko C phi gama
    
    uk = input(1);
    yk = input(2);
    
    xobs = (eye(2)-Ko*C)*phi*xobs + (eye(2)-Ko*C)*gama*uk + Ko*yk;
    
    output = xobs;
    
end