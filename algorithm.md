
# Load raw data

Data = load_vp_si();

```
Data.Laser.ranges = double(LASER)/100; % [m]
Data.Laser.time = double(TLsr)/1000;   % [s]
Data.Control.ve = speed;               % [m/s]
Data.Control.alpha = steering;         % [rad]
Data.Control.time = time/1000;         % [s]
Data.Gps.x = Lo_m;                     % [m]
Data.Gps.y = La_m;                     % [m]
Data.Gps.time = timeGps/1000;          % [s]
```

# Initialization

ci =1;

t = min(Data.Laser.time(1), Data.Control.time(1));

map = init_map();

M = init_local_map();

set_threshold;

# For to scan all the data

```
for k=1:length(Data.Laser.time)

        while (Data.Control.time(ci) < Data.Laser.time(k))
        
                X_p = motion_model(u(ci), t); 
                
                ci++;
                
                if (new_XY_vertex): {add_XY_vertex; add_XY_edge}
                
        Z = detect_tree(laser_range);
        
        T = NICP(M, Z);
        
        X, M = T.*(X_p, M_p);
        
        if (new_vertex(X, M)): {add_SE2_vertex; add_SE2_edge}

```
## Closure detection

# Add vertex

write_vertex_to_file


