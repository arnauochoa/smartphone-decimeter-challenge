%% Example of Orekit

% javaaddpath(workspacePath);
% javaaddpath([workspacePath '/orekit/orekit-10.3.jar'])
% javaaddpath([workspacePath '/orekit/hipparchus-1.8-bin/hipparchus-core-1.8.jar'])
% javaaddpath([workspacePath '/orekit/hipparchus-1.8-bin/hipparchus-geometry-1.8.jar'])
% javaaddpath([workspacePath '/orekit/hipparchus-1.8-bin/hipparchus-ode-1.8.jar'])
% javaaddpath([workspacePath '/orekit/hipparchus-1.8-bin/hipparchus-fitting-1.8.jar'])
% javaaddpath([workspacePath '/orekit/hipparchus-1.8-bin/hipparchus-optim-1.8.jar'])

import org.orekit.gnss.*

% Read Rinex
% rnxPath = './data/training/corrections/OSR/2020-08-06-US-MTV-2/ARWD219W.20o';
% rnxFIS = java.io.FileInputStream(rnxPath);
% rnx = RinexLoader(rnxFIS, 'arw');

s = Frequency.G01;
s.getName
s.getMHzFrequency
s.getWavelength
s.getSatelliteSystem