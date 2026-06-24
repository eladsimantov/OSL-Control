# How to setup tests on the OSL
1. Turn on a Hotspot / Wifi. (Make sure to preconfig the Pi to connect automatically)
2. Turn on the RPi.
3. SSH into the RPi, and Pull latest code from GitHub (via "git pull" command). 
4. Turn on Sensors.
5. Turn on the Motor power supply.
6. Run test_adapters_real_time.py to make sure all is good.


## If motor or loadcell not reading or recieving signals:
1. Restart both of their power supplies
2. Change the USB2CAN port in the RPi.
