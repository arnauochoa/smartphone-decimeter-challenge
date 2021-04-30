%% Example of Orekit

javaaddpath(workspacePath);
javaaddpath([workspacePath '/orekit/target/orekit-11.0-SNAPSHOT.jar'])
% javaaddpath([workspacePath '/orekit/hipparchus-1.8-bin/hipparchus-clustering-1.8.jar'])
javaaddpath([workspacePath '/orekit/hipparchus-1.8-bin/hipparchus-core-1.8.jar'])
% javaaddpath([workspacePath '/orekit/hipparchus-1.8-bin/hipparchus-fft-1.8.jar'])
% javaaddpath([workspacePath '/orekit/hipparchus-1.8-bin/hipparchus-filtering-1.8.jar'])
javaaddpath([workspacePath '/orekit/hipparchus-1.8-bin/hipparchus-fitting-1.8.jar'])
javaaddpath([workspacePath '/orekit/hipparchus-1.8-bin/hipparchus-geometry-1.8.jar'])
% javaaddpath([workspacePath '/orekit/hipparchus-1.8-bin/hipparchus-migration-1.8.jar'])
javaaddpath([workspacePath '/orekit/hipparchus-1.8-bin/hipparchus-ode-1.8.jar'])
javaaddpath([workspacePath '/orekit/hipparchus-1.8-bin/hipparchus-optim-1.8.jar'])
% javaaddpath([workspacePath '/orekit/hipparchus-1.8-bin/hipparchus-samples-1.8.jar'])
% javaaddpath([workspacePath '/orekit/hipparchus-1.8-bin/hipparchus-stat-1.8.jar'])

import org.orekit.gnss.*
import org.orekit.gnss.navigation.*
import org.orekit.time.*
import org.hipparchus.util.*

% Read Rinex
rnxPath = './data/training/brdc/2020-08-06-US-MTV-2/BRDC00WRD_R_20202190000_01D_GN.rnx';
rnxFIS = java.io.FileInputStream(rnxPath);
NFParser = NavigationFileParser();
gpsNav = NFParser.parse(rnxFIS);

% s = Frequency.G01;
% s.getName
% s.getMHzFrequency
% s.getWavelength
% s.getSatelliteSystem