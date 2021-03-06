%% Example of Orekit
clear;
import org.orekit.gnss.*
import org.orekit.gnss.navigation.*
import org.orekit.propagation.analytical.gnss.data.*
import org.orekit.propagation.analytical.gnss.*
import org.orekit.time.*
import org.hipparchus.util.*

%% Configure Orekit. The file orekit-data.zip must be in current dir
DM = org.orekit.data.DataProvidersManager.getInstance();
crawler = org.orekit.data.ZipJarCrawler('orekit-data.zip');
DM.clearProviders()
DM.addProvider(crawler)

% Read Rinex
rnxPath = './data/training/brdc/2020-08-06-US-MTV-2/BRDC00WRD_R_20202190000_01D_GN.rnx';
rnxFIS = java.io.FileInputStream(rnxPath);
nfParser = NavigationFileParser();
gpsNav = nfParser.parse(rnxFIS);
g17NavigationMessage = gpsNav.getGPSNavigationMessages('G17');
g17Propagator = GNSSPropagatorBuilder(g17NavigationMessage.get(0)).build();
g17State = g17Propagator.propagate(g17NavigationMessage.get(0).getDate());
% getDate --> UTC 
% coordinates in inertial frame 

% s = Frequency.G01;
% s.getName
% s.getMHzFrequency
% s.getWavelength
% s.getSatelliteSystem