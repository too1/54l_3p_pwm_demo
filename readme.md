54L-3P-PWM-Demo
---------------

## Overview

This demo demonstrates a way to use the RiscV core in the nRF54L to run the PWM modules of the device, 
in order to generate 3 pairs of PWM signals for controlling a 3-phase brushless motor. 

The goal is to demonstrate how the application core can control the PWM generation running in the RiscV core, but at the moment everything is running locally in the RiscV. 

## Requirements

SDK: 

	- https://github.com/masz-nordic/zephyr/tree/vpr_soc_launcher
	
Supported boards: 

	- nRF54L15-PDK

## Running the demo:

The simplest way to run the demo is to flash the nRF54L15PDK using the bat script in the prebuilt_fw folder. 
In order for the script to work it is necessary to install the [nRF Command Line Tools](https://www.nordicsemi.com/Products/Development-tools/nRF-Command-Line-Tools/Download)

With the necessary tools in place, open a command line window and navigate to the prebuilt_fw folder. From here, run the flash_54l_pdk.bat script.

Once the script is completed, open two terminal windows and connect to the VCOM0 and VCOM1 terminals of the DK. In order to figure out which comport is assigned to either it is possible to run the following comand:
***nrfjprog --com***

The comport associated with VCOM1 should show an output like this: 

```C
*** Booting Zephyr OS build v1.12.0-72586-g29632b49bfdd ***

VPR 3P PWM generation test
Press one of the following keys to control the PWM generation:
  r - Reduce the duty cycle by STEPSIZE. Range 0-500
  t - Increase duty cycle by STEPSIZE.
  f - Increase step timing by STEPSIZE microseconds. Range 0-10000us
  g - Reduce step timing by STEPSIZE microseconds.
  v - Reduce STEPSIZE. Avaiable values are 1, 2, 5, 10, 20, 50, 100, 200, 500
  b - Increase STEPSIZE
  u - Reduce PWM dead time by two ticks. Each tick equals 62.5ns
  i - Increase PWM dead time by two ticks.
  Press any other key to repeat these instructions
```

Enter keys in this terminal to configure the demo, as explained in the help text. 

## TODO: 
	- Update the demo to use NCS v2.7.0 once available. 
	- Move the control of the PWM to the application core, to demonstrate communication between the two cores
