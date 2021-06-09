function rnxFileNames = convertCrxOsr(crxPath, targetPath)
    crxFiles = getValidDir(crxPath);
    for iCrx = 1:length(crxFiles)
        if strcmp(crxFiles{iCrx}(end-3:end), '.crx')
            crx2rnx(strcat(crxPath, crxFiles{iCrx}));
        end
    end
    movefile(strcat(crxPath, '*.rnx'), targetPath);
    rnxFileNames = getValidDir(strcat(targetPath, '*.rnx')); 
end