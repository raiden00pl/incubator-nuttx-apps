FOC open loop example
---------------------

This example has been designed demostrate use of the Nuttx FOC driver
and be as simple as possible. For this reason, the controller
works in the open loop configuration (no phase angle feedback).

Hardware setup
--------------

TODO:

Hardware protection
-------------------

This example has not implement any mechanism to protect your
power stage. This means that there is no no overtemeprature
protection, no overcurrent protection and no overvoltage protection.

Make sure that you power the hardware properly and provide current
limits on your own.

State thread
-------------

TODO:

Sample configurations for some commercially available motors
------------------------------------------------------------
The FOC PI current controller parameters can be obtained from the given  equations:
   Kp = ccb * Ls;
   pp = Rs / Ls;
   Ki = pp * Kp * T;

where:
     Kp  - PI proportional coefficient
     Ki  - PI integral coefficient
     Rs  - average phase serial resistance
     Ls  - average phase serial inductance
     pp  - pole plant
     ccb - current control bandwidth
     T   - sampling period

Odrive D6374 150KV
    p      = 7
    Rs     = 0.0254 Ohm
    Ls     = 8.73 uH
    i\_max = ?
    v\_max = ?
 
  Example configuration for f\_PWM = 20kHz, f\_work = 10 kHz, f\_notifier = 10kHz, ccb=1000:
    Kp = 0.0087
    Ki = 0.0025
 
Linix 45ZWN24-40 (PMSM motor dedicated for NXP FRDM-MC-LVMTR kit)
    p      = 2
    Rs     = 0.5 Ohm
    Ls     = 0.400 mH
    i\_max = 2.34 A
    v\_max = 24 V
 
  Example configuration for f\_PWM = 10kHz, f\_work = 5 kHz, f\_notifier = 5kHz, ccb=1000:
    Kp = 0.4
    Ki = 0.1
 
Bull-Running BR2804-1700 kV (Motor provided with the ST P-NUCLEO-IHM07 kit)
    p      = 7
    Rs     = 0.11 Ohm
    Ls     = 0.018 mH
    i\_max = 1.2A
    v\_max = 12V
 
  Example configuration for f\_PWM = 20kHz, f\_work = 10 kHz, f\_notifier = 10kHz, ccb=200:
    Kp = 0.036
    Ki = 0.022
 
iPower GBM2804H-100T (Gimbal motor provided with the ST P-NUCLEO-IHM03 kit)
    p      = 7
    Rs     = 5.29 Ohm
    Ls     = 1.05 mH
    i\_max = 0.15A
    v\_max = 12V
 
  Example configuration for f\_PWM = 10kHz, f\_work = 5 kHz, f\_notifier = 5kHz, ccb=TODO:
    Kp = TODO
    Ki = TODO
 

