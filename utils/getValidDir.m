function dirNames = getValidDir(path)

invalidDir = {'.', '..'};


dirNames = {dir(path).name};
idxInvalid = zeros(size(dirNames));

for iInvalid = 1:length(invalidDir)
    idxInvalid = idxInvalid | strcmp(dirNames, invalidDir{iInvalid});
end
dirNames(idxInvalid) = [];

end