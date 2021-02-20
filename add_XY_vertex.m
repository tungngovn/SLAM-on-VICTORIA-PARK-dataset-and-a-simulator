function add_XY_vertex(X)

global G2O;
global States;

% size(G2O.XY_vertex,2)
% G2O.XY_vertex(4,1)

i = G2O.XY_vertex(4,size(G2O.XY_vertex,2));
new_vertex = [X; i+1];
States.vertex = new_vertex;
G2O.XY_vertex = [G2O.XY_vertex new_vertex];