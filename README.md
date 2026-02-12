# OSL-Control
A repo to test control strategies on the OSLv2 using ODrive and CubeMars motors in the eNaBLe lab.

# Temporary example
## Instructions to run example
Turn on motors (connect cable, check battery 100%)
run can example
o - turn from idle to closed loop

<!-- ## Example
===== Dual-Motor Walking Controller =====
o              - closed loop both motors
f              - idle both motors
p1<p>          - knee by <p> degrees (relative)
p2<p>          - ankle by <p> degrees (relative)
v1<v>          - knee velocity
v2<v>          - ankle velocity
walk           - start walking
stop           - stop walking
ep             - print encoder positions (both)
dump1/dump2    - dump raw/decoded messages for axis
phase <deg>    - set ankle phase relative to knee (deg)
phaseboth k a  - set both phases (deg)
phaseinc k a   - increment phases by k,a degrees
phasereset     - reset phases to defaults (knee 0°, ankle 180°)
cal1 <s>       - calibrate knee (optional seconds)
cal2 <s>       - calibrate ankle (optional seconds)
save1          - save knee calibration params
save2          - save ankle calibration params
x              - exit -->



# Cascade control architecture for Odrive motors 
https://docs.odriverobotics.com/v/latest/manual/control.html

the torque is in N-m

# Attention
The voltage of the battery (18.5V) is lower than the motor (AK80-9) rated voltage.

In the first try we need to run the sudo line.

# updates
important basic functions:
1. set state - idle,closed loop, calibration 
2. control mode - for changing between torque,position and velocity

# Tests
There is a test script to run position, torque and impedance control 

There are functions to read the current (Q axis) and position sensors.

`test_impedance_knee.py`

# Torque control via Odrive
Odrive compute the torque based on the back-emf constant via the equation $K_t=8.27/K_v=0.0827[Nm/A]$. However, our motor datasheet gives its own estimate of $K_t=0.095[Nm/A]$. This value is at the motor output (NOT the JOINT!)  

The reduction ratios are twofold. $N=9:1$ at the motor planetary gear, and for the actual leg joints there is a further 5.44 belt drive reduction. 

# The open source leg
See specifications-https://opensourceleg.org/hardware
They give the 41.5:1 reduction from joint level to motor revolution level. 
Also they give a torque constant of 0.14 Nm/A for an actuator with 0.112 Nm/A.

# Hardware
## Motors 
We are using the TKmotor (CubeMars) AK80-9 BLDC motors 
([see here](https://www.cubemars.com/product/ak80-9-v3-0-robotic-actuator.html)). Or in depth AK series [here](https://www.cubemars.com/images/file/20241217/1734428582786471.pdf).

## Encoders
We are using AMT102-V incremental encoders
([see here](https://www.sameskydevices.com/product/resource/amt10.pdf)).
See also the [Mounting guide](https://www.sameskydevices.com/resources/resource-library/videos/mounting-the-amt10-and-10e-modular-encoder-series).


## LoadCell
We are using Sunrise Instruments Loadcell with its OEM board. 