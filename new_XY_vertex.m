function new = new_XY_vertex(X_ab)

global Threshold;
global States;

Threshold.vertex_translation

if (translation(X_ab, States.vertex) > Threshold.vertex_translation)
    new = 1;
else
    new = 0;
end

function distance = translation(X, Y)
distance = sqrt((X(1)-Y(1))^2 + (X(2)-Y(2))^2);




