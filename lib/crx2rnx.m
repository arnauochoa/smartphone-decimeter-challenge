function crx2rnx(rnxFilepath)
    scriptPath = [projectPath 'lib' filesep 'RNXCMP_4.0.8' filesep 'bin' filesep 'CRX2RNX'];
    cmd = strcat(scriptPath, " ", rnxFilepath, ' -f');
    [status,cmdout] = system(cmd);
    if status ~= 0
        error(strcat('CRX2RNX failed:', cmdout));
    end
end