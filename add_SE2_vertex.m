function add_SE2_vertex(X)

global G2O;
global States;

i = G2O.SE2_vertex(4,size(G2O.SE2_vertex,2));
new_vertex = [X; i+1];
States.vertex = new_vertex;
G2O.SE2_vertex = [G2O.SE2_vertex new_vertex];