from src.orchestrator import Orchestrator
from src.classifier import Classification

import time

def main():

    o = Orchestrator()
    o.connect()
    takeoff = False

    try:

        while(1):
            classification, position = o.loop()
            joint_positions = o.run_calculations()

            cmds = []
            if classification == Classification.WRIST_FLEX_TURN_LEFT:
                cmds.append((f"FC:YAW:LEFT\n", 5, True, "END_RESPONSE"))
            elif classification == Classification.WRIST_EXT_TURN_RIGHT:
                cmds.append((f"FC:YAW:RIGHT\n", 5, True, "END_RESPONSE"))
            elif classification == Classification.WRIST_ADD_ARM_DOWN:
                cmds.append((f"AR:SERVO:0:{joint_positions[0]}\n", 3, True, "END_RESPONSE"))
                cmds.append((f"AR:SERVO:1:{joint_positions[1]}\n", 3, True, "END_RESPONSE"))
            elif classification == Classification.WRIST_ABD_ARM_UP:
                cmds.append((f"AR:SERVO:0:{joint_positions[0]}\n", 3, True, "END_RESPONSE"))
                cmds.append((f"AR:SERVO:1:{joint_positions[1]}\n", 3, True, "END_RESPONSE"))

            if classification == Classification.GRASP_SPRAY:
                cmds.append((f"AR:PUMP:ON\n", 3, True, "END_RESPONSE"))
            else:
                cmds.append((f"AR:PUMP:OFF\n", 3, True, "END_RESPONSE"))
            

            # TODO: update this with the actual thresholds
            if position[1] is not None:
                if not takeoff and position[1].z > 0.5: 
                    cmds.append((f"FC:TAKEOFF:{o.drone.takeoff_alt}\n", 15, True, "END_RESPONSE"))
                    takeoff = True
                if takeoff and position[1].y > 0.5:
                    cmds.append((f"FC:MOVE:FWD\n", 5, True, "END_RESPONSE"))
                if takeoff and position[1].y < -0.5:
                    cmds.append((f"FC:MOVE:BACK\n", 5, True, "END_RESPONSE"))
                if takeoff and position[1].z < -0.5:
                    cmds.append(("FC:LAND\n", 10, True, "END_RESPONSE"))
                    takeoff = False

            o.send_output(cmds)
            time.sleep(o.polling_period)
    except KeyboardInterrupt:
        print("Interrupt Caught. Exiting.")
    finally:
        o.disconnect()
        
    
if __name__ == '__main__':
    main()