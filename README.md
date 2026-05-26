# OSL-Control
A repo to test control strategies on the OSLv2 using ODrive and CubeMars motors in the eNaBLe lab.


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

https://technionmail-my.sharepoint.com/:w:/g/personal/elad_sim_campus_technion_ac_il/IQB34CwJhmjCRr4ecyUJv1SMATLv-zqkLTahnImUDXmteCA?e=xWJupW
v/bin/activate  # On Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

### 2. Running Unit Tests
Validate the mathematics, coordinate transformations, and filtering algorithms:
```bash
python -m unittest tests/test_enabletools.py
```

### 3. Running Hardware Tests (Online)
To test the real-time loop and hardware adapters:
```bash
# Real-time ODrive torque control and loadcell polling
sudo python tests/test_adapters_real_time.py

# IMU reading test
python tests/test_imu.py
```

---

## 🔍 Debugging & Troubleshooting

### 1. CAN Bus Debugging (ODrive & Loadcell)
If your CAN device is not responding:
* **Reset the CAN interface:**
  ```bash
  sudo ip link set can0 down
  sudo ip link set can0 up type can bitrate 1000000 sample-point 0.750
  sudo ip link set can0 txqueuelen 1000
  ```
* **Monitor raw CAN traffic:**
  ```bash
  candump can0
  ```
* **Verify bitrate:** Make sure both the ODrive and the Loadcell board are configured for `1000000` (1 Mb/s).

### 2. I2C Bus Debugging (IMU)
If the BNO055 IMU initialization fails:
* **Scan the I2C bus:**
  ```bash
  i2cdetect -y 1
  ```
* **Address Check:** The BNO055 default address is usually `0x28`. If `i2cdetect` shows `29`, make sure to instantiate the adapter with `addr=0x29`.

### 3. Keyboard Interrupts / Ctrl+C
* The `SoftRealtimeLoop` class catches `SIGINT` signals internally via its `LoopKiller` module. This means a standard `except KeyboardInterrupt` block will **not** trigger inside the loop.
* **Solution:** Always perform cleanup tasks (like moving the ODrive to `idle` mode or bringing the CAN interface down) in a `finally:` block, which runs regardless of how the loop exits.
