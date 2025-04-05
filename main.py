from src.orchestrator import Orchestrator

def main():

    o = Orchestrator()
    o.connect()
    takeoff = False

    try:
        while(1):
            classification, position = o.poll_sensors()

            cmds = []

            # TODO: update this with he actual classifications and servo values
            if classification == "":
                cmds.append((f"FC:YAW:LEFT\n", 5, True, "END_RESPONSE"))
            elif classification == "":
                cmds.append((f"FC:YAW:RIGHT\n", 5, True, "END_RESPONSE"))
            elif classification == "":
                cmds.append((f"AR:PUMP:ON\n", 3, True, "END_RESPONSE"))
            elif classification == "":
                cmds.append((f"AR:PUMP:OFF\n", 3, True, "END_RESPONSE"))
            elif classification == "":
                joint_positions = o.run_calculations(0)
                cmds.append((f"AR:SERVO:0:{joint_positions[0]}\n", 3, True, "END_RESPONSE"))
                cmds.append((f"AR:SERVO:1:{joint_positions[1]}\n", 3, True, "END_RESPONSE"))
            elif classification == "":
                joint_positions = o.run_calculations(0)
                cmds.append((f"AR:SERVO:0:{joint_positions[0]}\n", 3, True, "END_RESPONSE"))
                cmds.append((f"AR:SERVO:1:{joint_positions[1]}\n", 3, True, "END_RESPONSE"))
            

            # TODO: update this with the actual thresholds
            if not takeoff and position > 5: 
                cmds.append((f"FC:TAKEOFF:{o.drone.takeoff_alt}\n", 15, True, "END_RESPONSE"))
                takeoff = True
            if takeoff and position > 5:
                cmds.append((f"FC:MOVE:FWD\n", 5, True, "END_RESPONSE"))
            if takeoff and position < 5:
                cmds.append((f"FC:MOVE:BACK\n", 5, True, "END_RESPONSE"))
            if takeoff and position < 5:
                cmds.append(("FC:LAND\n", 10, True, "END_RESPONSE"))
                takeoff = False

    finally:
        o.disconnect()
        
    
if __name__ == '__main__':
    main()