import can
import struct
import time

# הגדרות בסיסיות
NODE_ID = 0  # ה-Node ID שהוגדר ב-ODrive (ברירת מחדל 0)
INTERFACE = 'can0' # או 'socketcan', תלוי במתאם שלך

# מזהי פקודות (Command IDs) מתוך ה-Documentation של ODrive
CMD_HEARTBEAT_MSG = 0x001
CMD_SET_AXIS_STATE = 0x007
CMD_SET_INPUT_POS = 0x00C
CMD_CLEAR_ERRORS = 0x018

# מצבי מערכת (Axis States)
AXIS_STATE_IDLE = 1
AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3
AXIS_STATE_CLOSED_LOOP_CONTROL = 8

def send_can_command(bus, node_id, cmd_id, data=None, format=''):
    """פונקציית עזר לשליחת הודעת CAN"""
    can_id = (node_id << 5) | cmd_id
    if data is not None:
        packed_data = struct.pack(format, *data)
    else:
        packed_data = None
    
    msg = can.Message(arbitration_id=can_id, data=packed_data, is_extended_id=False)
    try:
        bus.send(msg)
    except can.CanError:
        print("שגיאה בשליחת הודעת CAN")

def main():
    # 1. חיבור לאוטובוס ה-CAN
    # וודא שה-Baudrate תואם להגדרות ב-ODrive (למשל 250k או 500k)
    bus = can.interface.Bus(channel=INTERFACE, bustype='socketcan')

    print("מנקה שגיאות...")
    send_can_command(bus, NODE_ID, CMD_CLEAR_ERRORS)
    time.sleep(0.5)

    # 2. הפעלת כיול (אם המנוע לא מכויל, הוא לא יעבור לירוק)
    print("מתחיל תהליך כיול... המנוע אמור לזוז.")
    send_can_command(bus, NODE_ID, CMD_SET_AXIS_STATE, [AXIS_STATE_FULL_CALIBRATION_SEQUENCE], '<I')
    
    # המתנה לסיום הכיול (בד"כ לוקח 10-15 שניות)
    time.sleep(15)

    # 3. מעבר למצב Closed Loop (כאן הנורית אמורה להפוך לירוקה)
    print("עובר למצב Closed Loop (נורית ירוקה)...")
    send_can_command(bus, NODE_ID, CMD_SET_AXIS_STATE, [AXIS_STATE_CLOSED_LOOP_CONTROL], '<I')
    time.sleep(1)

    # 4. שליחת פקודת מיקום (דוגמה: סיבוב למיקום 2.0)
    print("שולח פקודת תנועה...")
    # מבנה: Position (float), VelocityFF (int16), TorqueFF (int16)
    position = 2.0
    send_can_command(bus, NODE_ID, CMD_SET_INPUT_POS, [position, 0, 0], '<fhh')

if __name__ == "__main__":
    main()