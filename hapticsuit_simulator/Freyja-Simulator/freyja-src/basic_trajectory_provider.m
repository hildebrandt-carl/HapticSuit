%BASIC_TRAJECTORY_PROVIDER Example circular trajectory for multicopter.
%
% Usage notes:
%     > Simulink allows only one input (a Matlab vector, prefer 1D)
%     > Use vector-concatenate block in Simulink to have this input contain
%       more elements, and then parse it here -- perhaps like so:
%         tnow        = func_input(1);
%         n_cookies   = func_input(11);
%         my_state    = func_input(2:8);      % [pn;pe;pd;vn;ve;vd;yaw]
%         other_data  = func_input(9:10);
%
%     > Output must always be a 7-vector reference: [pn;pe;pd;vn;ve;vd;yaw]
%     > Use `assert(length(TREF))==7` if using codegen.
%
%  ~ aj / Nimbus Lab.
function TREF = basic_trajectory_provider( func_input )

tnow = func_input(1);

% ---------------------------
% Circle Trajectory
% ---------------------------
% TREF = [ 2*sin(tnow);  ...
%          2*cos(tnow);  ...
%          -1.5;         ...
%          2*cos(tnow);  ...
%          -2*sin(tnow); ...
%          0;            ...
%          0 ];





% ---------------------------
% Line Trajectory
% ---------------------------
if mod(tnow, 4) < 2
    vel = 2 ;
    current_tnow = tnow;
    while current_tnow > 2
        current_tnow = current_tnow - 4 ;
    end
else
    vel = -2 ;
    current_tnow = 4 - tnow;
    while current_tnow < 0
        current_tnow = current_tnow + 4;
    end
end
    
TREF = [ 2*(current_tnow);  ...
         0;  ...
         -1.5;         ...
         vel;  ...
         0; ...
         0;            ...
         0 ];


% ---------------------------
% Hover
% ---------------------------
% TREF = [ 0;  ...
%          0;  ...
%          -3;         ...
%          0;  ...
%          0; ...
%          0;            ...
%          0 ];     
end

