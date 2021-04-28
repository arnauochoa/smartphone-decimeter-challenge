function [path] = workspacePath()
% WORKSPACEPATH returns the workspace path
[utilsPath, ~, ~]   = fileparts(mfilename('fullpath'));
[projectPath, ~, ~] = fileparts(utilsPath);
[path, ~, ~]        = fileparts(projectPath);
path = [path filesep];
end
