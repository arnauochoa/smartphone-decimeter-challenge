# Smartphone decimeter challenge
Matlab package for the Google Smartphone Decimeter Challenge at ION GNSS+ 2021.

## Installation
This code uses the following external packages:
* [EKF](https://github.com/jtec/EKF)
* [INS](https://github.com/jtec/INS)
* [magnitude](https://redmine.recherche.enac.fr/projects/magnitude/repository)
* [android-measurements](https://github.com/arnauochoa/android-measurements)
* [gps-measurement-tools/NmeaUtils](https://github.com/google/gps-measurement-tools)
* [orekit](http://www.orekit.org/download.html). See [this tutorial](https://www.orekit.org/site-orekit-tutorials-10.3/tutorials/integration-in-other-languages.html) on how to integrate Orekit within MATLAB.

## Usage
* Make sure you have the input data saved in your workspace with the following structure:
```bash
Observations:   {workspace_path}/data/training/datasets/{campaign_name}/{phone_name}_GnssLog.txt
Groundtruth:    {workspace_path}/data/training/datasets/{campaign_name}/SPAN_{phone_name}_10Hz.nmea
Navigation:     {workspace_path}/data/training/brdc/{campaign_name}/BRDC00WRD_R_{datetime}_01D_GN.rnx
```
* Set your workspace as the parent directory that contains all the packages.
* Select the desired configuration in the class `Config`.
* Run `main`.
