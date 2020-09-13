# ESP-IDF

## Installation and tutorials:

https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html#step-5-start-a-project



Initiate directory with 

```get_idf```

then for the first time

```
idf.py set-target esp32
idf.py menuconfig

```

then

```
idf.py build
idf.py -p /dev/tty.SLAB_USBtoUART flash
idf.py -p /dev/tty.SLAB_USBtoUART monitor
idf.py -p /dev/tty.SLAB_USBtoUART flash monitor

```


tips

* use ESP_LOGD and use menuconfig to define log level in component configuration of LOG
* to get out of monitor: CTRL+]



# Programs

## pca9685_pwm_test 

This can be used to test and determin the angles of the servos. it will create two array (servo min and factor which can be used in the main program)



## ik_test

This is a program in development to test the current Inverse Kinetics. It will calculate and move the servos based on the omega, phi and psi angles and xm,ym and zm position of the body. No gait is introduced in this phase.
The angles can be entered using a cordova program or in code.

Make sure to init and update the git submodules to include ESP DSP: https://github.com/espressif/esp-dsp.git





