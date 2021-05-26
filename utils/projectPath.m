function [path] = projectPath()
% PROJECTPATH returns the project path
[utilsPath, ~, ~]   = fileparts(mfilename('fullpath'));
[path, ~, ~] = fileparts(utilsPath);
path = [path filesep];
end
