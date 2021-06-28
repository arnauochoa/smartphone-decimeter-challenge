function crx2rnx(rnxFilepath)
    if ispc, rnxcmp_folder = 'RNXCMP_4.0.8_windows'; end
    if isunix, rnxcmp_folder = 'RNXCMP_4.0.8_linux'; end
    scriptPath = [projectPath 'lib' filesep rnxcmp_folder filesep 'bin' filesep 'CRX2RNX'];
    cmd = strcat(scriptPath, " ", rnxFilepath, ' -f');
    [status,cmdout] = system(cmd);
    if status ~= 0
        error(strcat('CRX2RNX failed:', cmdout));
    end
end