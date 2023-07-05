# Smartphone decimeter challenge
Matlab package for the Google Smartphone Decimeter Challenge at ION GNSS+ 2021.

This project was part of my Master's Thesis at ENAC (École Nationale de l'Aviation Civile).
The thesis report and presentation can be found in [master_thesis](master_thesis) and the paper can be downloaded [here](https://enac.hal.science/hal-03450489/).

## Installation
This code uses the following external packages:
* __Submodules__
    * [EKF](https://github.com/jtec/EKF) (private)
    * [INS](https://github.com/jtec/INS) (private)
    * [magnitude](https://redmine.recherche.enac.fr/projects/magnitude/repository) (private)
    * [android-measurements](https://github.com/arnauochoa/android-measurements)
    * [gps-measurement-tools/NmeaUtils](https://github.com/google/gps-measurement-tools)
* ___Yet not supported___
    * [Orekit](http://www.orekit.org/download.html). See [this tutorial](https://www.orekit.org/site-orekit-tutorials-10.3/tutorials/integration-in-other-languages.html) on how to integrate Orekit within MATLAB.

## Usage
* Make sure you have the input data saved in your workspace with the following structure:
```bash
Observations:   {workspace_path}/data/sdc-data/{train or test}/{campaign_name}/{phone_name}_GnssLog.txt
Groundtruth:    {workspace_path}/data/sdc-data/train/{campaign_name}/SPAN_{phone_name}_10Hz.nmea
Navigation:     {workspace_path}/data/sdc-data/brdc/{campaign_name}/BRDC00WRD_R_{datetime}_01D_GN.rnx
Verizon OSR:    {workspace_path}/data/sdc-data/corrections/Verizon/OSR/{campaign_name}/{OSR_filename}.rnx
SwiftNav OSR:   {workspace_path}/data/sdc-data/corrections/SwiftNav/OSR/{OSR_filename}.obs
```
> :warning: &nbsp; **Make sure to have the OSR RINEX files in version 3.03 or above.**
* Set your workspace as the parent directory that contains this project. Your workspace should look like the following:
```bash
/some/directory/
        |-- data/
                |-- sdc-data/
                        |-- brdc
                        |-- corrections
                        |-- train
                        |-- test
        |-- smartphone-decimeter-challenge/
```
* Select the desired configuration in the class `Config`.
* Run `main`.
