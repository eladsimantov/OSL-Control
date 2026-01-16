# OSL-Control
A repo to test control strategies on the OSLv2 using ODrive and CubeMars motors in the eNaBLe lab.

# Temporary example
## Instructions to run example
Turn on motors (connect cable, check battery 100%)
run can example
o - turn from idle to closed loop

## Example
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
x              - exit



# Cascade control architecture for Odrive motors 
https://docs.odriverobotics.com/v/latest/manual/control.html

the torque is in N-m



# updates
important basic functions:
1. set state - idle,closed loop, calibration 
2. control mode - for changing between torque,position and velocity