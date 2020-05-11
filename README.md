# COMPSYS 723 Assignment 1 - Load Control System

Import the root directory as an existing project into your NIOS II workspace, making sure to include both projects (`Assignment1` and `Assignment1_bsp`). This should import both the assignment project as well as the BSP. The BSP has only been modified to allow TIMER1_US to be used as the timestamp timer.

SW4-SW0 simulate the switches for the loads, with LEDR4-LEDR0 representing the system output for each loud respectively. LEDG4-LEDG0 represent whether a load has currently been switched off by the load management system.

KEY3 will toggle the system between managing loads and maintenance mode.

While in maintenance mode, the threshold values for both instantaneous frequency and rate of change can be set.  Each threshold can only have 3 digits, and is in the format  XX.X, ranging from 00.0 to 99.9. The first three numbers typed will set the instant threshold, and the next 3 will set the rate of change. This cycle repeats indefinitely. The thresholds are updated instantly, and the number that is seen on the GUI is the actual value of the threshold, should you switch back to managing loads.
As an example, switching to maintenance mode, then press  5 0 0 1 1 0 will set the instant threshold to 50.0 Hz and the ROC to 11.0 Hz/sec
