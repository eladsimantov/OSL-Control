"""
LocolabPhaseVariableExample.py

This is the Locolab's Phase Variable Module for the Open Source Leg.

Note: You must have the LocolabPhaseVariable.so library available for this script to run.
    Place it in either the same directory or update the 'library_path' argument below.

T. K. Best, C. G. Welker, E. H. Cimino, and R. D. Gregg
6/26/2024

"""

import os
import inspect
import numpy as np
from opensourceleg.osl import OpenSourceLeg
from opensourceleg.control.compiled_controller import CompiledController
from opensourceleg.hardware.sensors import IMULordMicrostrain
from opensourceleg.tools import units

###### CONFIGURATION ######
LOADCELL_MATRIX = np.array(
    [
        (-38.72600, -1817.74700, 9.84900, 43.37400, -44.54000, 1824.67000),
        (-8.61600, 1041.14900, 18.86100, -2098.82200, 31.79400, 1058.6230),
        (-1047.16800, 8.63900, -1047.28200, -20.70000, -1073.08800, -8.92300),
        (20.57600, -0.04000, -0.24600, 0.55400, -21.40800, -0.47600),
        (-12.13400, -1.10800, 24.36100, 0.02300, -12.14100, 0.79200),
        (-0.65100, -28.28700, 0.02200, -25.23000, 0.47300, -27.3070),
    ]
)
LOOP_FREQUENCY = 300  #Hz
LOG_NAME = './phase_var'
WALKING_SPEED = 1.0
WALKING_INCLINE = 0.0
##########################


osl = OpenSourceLeg(frequency=LOOP_FREQUENCY, file_name = LOG_NAME)
osl.clock.report = True

osl.add_loadcell(joint=osl.knee, loadcell_matrix=LOADCELL_MATRIX)

thigh_imu = IMULordMicrostrain(port=r"/dev/serial0", sample_rate=LOOP_FREQUENCY)

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

PhaseVariable = CompiledController(
    library_name="LocolabPhaseVariable",
    library_path=currentdir,
    main_function_name="LocolabPhaseVariable",
    initialization_function_name="LocolabPhaseVariable_initialize",
    cleanup_function_name="LocolabPhaseVariable_terminate",
)


PhaseVariable.define_inputs(
    [
        ("thighAngle_deg", PhaseVariable.types.c_double),
        ("thighVelocity_dps", PhaseVariable.types.c_double),
        ("Fz", PhaseVariable.types.c_double),
        ("time", PhaseVariable.types.c_double),
        ("incline", PhaseVariable.types.c_double),
        ("speed", PhaseVariable.types.c_double),
    ]
)


PhaseVariable.define_outputs(
    [
        ("phase", PhaseVariable.types.c_double),
        ("stancePhase", PhaseVariable.types.c_double),
        ("swingPhase", PhaseVariable.types.c_double),
        ("state", PhaseVariable.types.c_double),
    ]
)


PhaseVariable.inputs.speed = WALKING_SPEED       
PhaseVariable.inputs.incline = WALKING_INCLINE     

thigh_imu.start_streaming()

osl.log.add_attributes(osl.loadcell, ["fz"])
osl.log.add_attributes(PhaseVariable.outputs, ["phase", "stancePhase", "swingPhase", "state"])
osl.log.add_attributes(thigh_imu.imu_data, ["angle_x"])

with osl:
    for time in osl.clock:
        osl.update()
        imu_data = thigh_imu.get_data()
        PhaseVariable.inputs.thighAngle_deg = units.convert_from_default((imu_data.angle_x + 0.6873), units.position.deg)  # add hardware mounting offset
        PhaseVariable.inputs.thighVelocity_dps = units.convert_from_default(imu_data.velocity_x, units.velocity.deg_per_s)
        PhaseVariable.inputs.Fz = osl.loadcell.fz
        PhaseVariable.inputs.time = time
      
        outputs = PhaseVariable.run()
        print("Phase: {:.2f}, Stance Phase: {:.2f}, Swing Phase: {:.2f}, State: {:.2f}.".format(
            outputs.phase, outputs.stancePhase, outputs.swingPhase, outputs.state
        ), end="\r")

thigh_imu.stop_streaming()
print()





