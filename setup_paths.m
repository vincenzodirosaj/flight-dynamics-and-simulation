function setup_paths
%SETUP_PATHS Make the original scripts and their dependencies accessible.
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root,'HARV_6DoF'),fullfile(root,'HARV_6DoF','Aerodynamics'), ...
    fullfile(root,'HARV_6DoF','Additional_Maneuver'),fullfile(root,'Speed_Brakes'));
end
