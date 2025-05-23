from src.orchestrator import Orchestrator
from src.classifier import Classification

import time
import sys
from numpy import pi

def main():

    o = Orchestrator()
    o.connect()
    takeoff = False
    land = False

    try:

        while(1):
            classification, position = o.loop()
            joint_positions = o.run_calculations()
            joint_positions[1] = (-joint_positions[1]*180/pi)
            joint_positions[2] = (-joint_positions[2]*180/pi)

            cmds = []
            if classification == Classification.WRIST_FLEX_TURN_LEFT:
                cmds.append((f"FC:YAW:LEFT\n", 5, "END_RESPONSE"))
            elif classification == Classification.WRIST_EXT_TURN_RIGHT:
                cmds.append((f"FC:YAW:RIGHT\n", 5, "END_RESPONSE"))
            elif classification == Classification.WRIST_ADD_ARM_DOWN:
                cmds.append((f"AR:SERVO:PITCH:{joint_positions[1]}\nAR:SERVO:PITCH2:{joint_positions[2]}\n", 3, "END_RESPONSE"))
                # cmds.append((f"AR:SERVO:PITCH2:{joint_positions[2]}\n", 3, "END_RESPONSE"))
            elif classification == Classification.WRIST_ABD_ARM_UP:
                cmds.append((f"AR:SERVO:PITCH:{joint_positions[1]}\nAR:SERVO:PITCH2:{joint_positions[2]}\n", 3, "END_RESPONSE"))
                # cmds.append((f"AR:SERVO:PITCH2:{joint_positions[2]}\n", 3, "END_RESPONSE"))

            if classification == Classification.GRASP_SPRAY:
                cmds.append((f"AR:PUMP:ON\n", 3, "END_RESPONSE"))
            # else:
            #     cmds.append((f"AR:PUMP:OFF\n", 3, "END_RESPONSE"))
            

            # TODO: update this with the actual thresholds
            if position[1] is not None:
                if not takeoff and position[1].y > 0.45: 
                    cmds.append((f"FC:TAKEOFF:{o.drone.takeoff_alt}\n", 15, "END_RESPONSE"))
                    takeoff = True
                if takeoff and position[1].x > 0.4:
                    cmds.append((f"FC:MOVE:FWD\n", 5, "END_RESPONSE"))
                if takeoff and position[1].x < 0.2:
                    cmds.append((f"FC:MOVE:BACK\n", 5, "END_RESPONSE"))
                if takeoff and position[1].y < -0.25:
                    cmds.append(("FC:LAND\n", 10, "END_RESPONSE"))
                    land = True

            o.send_output(cmds)
            if land:
                sys.exit(1)
            time.sleep(o.polling_period)
    except KeyboardInterrupt:
        print("Interrupt Caught. Exiting.")
    finally:
        o.disconnect()
        
    
if __name__ == '__main__':
    main()