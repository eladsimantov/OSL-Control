import pigpio, time
from collections import deque

PIN = 17
AVG_N = 10    # moving average window

pi = pigpio.pi()
if not pi.connected:
    raise SystemExit("pigpio not running")

last_rise = None
last_period = None
highs = deque(maxlen=AVG_N)
duties = deque(maxlen=AVG_N)

def cb(gpio, level, tick):
    global last_rise, last_period
    if level == 1:  # rising
        if last_rise is not None:
            last_period = pigpio.tickDiff(last_rise, tick)
        last_rise = tick
    elif level == 0 and last_rise is not None:  # falling
        high = pigpio.tickDiff(last_rise, tick)  # microseconds
        if last_period:
            duty = high / last_period
            highs.append(high)
            duties.append(duty)

cb = pi.callback(PIN, pigpio.EITHER_EDGE, cb)

try:
    while True:
        if duties:
            avg_high = sum(highs) / len(highs)
            avg_duty = sum(duties) / len(duties)
            print(f"High(us): {avg_high:.1f}, Duty: {avg_duty*100:.2f}%, Period(us): { (avg_high/(avg_duty) if avg_duty>0 else 0):.0f }")
        time.sleep(0.05)
except KeyboardInterrupt:
    pass
finally:
    cb.cancel()
    pi.stop()
