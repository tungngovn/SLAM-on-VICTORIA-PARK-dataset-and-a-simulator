function write_g2o_file(vertex)

vertex;
fileID = fopen('victoria_park.g2o','w');
g2o_matrix = zeros(size(vertex));
g2o_matrix(1,:) = vertex(4,:);
g2o_matrix(2,:) = vertex(1,:);
g2o_matrix(3,:) = vertex(2,:);
g2o_matrix(4,:) = vertex(3,:);
fprintf(fileID, 'VERTEX_SE2 %d %f %f %f\n', g2o_matrix);

edge = zeros(5,1);
for i=1:(length(vertex)-1)
    vertex(:,i+1);
    diff = vertex(:,i+1)-vertex(:,i)
    edge_c = [i; i+1; diff(1:3,1)];
    edge = [edge edge_c];
end

fprintf(fileID, 'EDGE_SE2 %d %d %f %f %f\n', edge);