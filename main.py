from src.orchestrator import Orchestrator
from src.classifier import Classification
from src.drone_arm import DroneArm
from spatialmath import *
import math

import time

def main():

    o = Orchestrator()
    o.connect()
    takeoff = False

    try:
        
        drone_arm = DroneArm()
        current_joint_angles = drone_arm.qn
        current_arm_pose = drone_arm.get_current_pose()

        while(1):
            classification, position = o.loop()

            cmds = []

            # handle myo band classification response
            pose_modifier = o.myoband.get_pose_modifier()
            modified_pose = pose_modifier * current_arm_pose
            ik_res = drone_arm.get_ikine(modified_pose)

            if modified_pose != current_arm_pose and ik_res.success and modified_pose.t[2] < -0.1:
                # we have updates to send
                drone_angle_diff = ik_res.q[0] - current_joint_angles[0]
                if (abs(drone_angle_diff) > 0.1):
                    #send a heading change
                    #this should work for both left and right since the command interpreter
                    #in the drone code seems to just throw a negative sign on the angle if 
                    #Left is specified
                    cmds.append((f"FC:YAW:LEFT\n", drone_angle_diff, True, "END_RESPONSE"))
                else:
                    #send arm servo changes
                    cmds.append((f"AR:SERVO:0:{math.degrees(ik_res.q[1])}\n", 3, True, "END_RESPONSE"))
                    cmds.append((f"AR:SERVO:1:{math.degrees(ik_res.q[2])}\n", 3, True, "END_RESPONSE"))

                current_arm_pose = modified_pose
                current_joint_angles = ik_res.q
                print('sent drone stuff')
            # if classification == Classification.WRIST_FLEX_TURN_LEFT:
            #     cmds.append((f"FC:YAW:LEFT\n", 5, True, "END_RESPONSE"))
            # elif classification == Classification.WRIST_EXT_TURN_RIGHT:
            #     cmds.append((f"FC:YAW:RIGHT\n", 5, True, "END_RESPONSE"))
            # elif classification == Classification.WRIST_ADD_ARM_DOWN:
            #     joint_positions = o.run_calculations(0)
            #     cmds.append((f"AR:SERVO:0:{joint_positions[0]}\n", 3, True, "END_RESPONSE"))
            #     cmds.append((f"AR:SERVO:1:{joint_positions[1]}\n", 3, True, "END_RESPONSE"))
            # elif classification == Classification.WRIST_ABD_ARM_UP:
            #     joint_positions = o.run_calculations(0)
            #     cmds.append((f"AR:SERVO:0:{joint_positions[0]}\n", 3, True, "END_RESPONSE"))
            #     cmds.append((f"AR:SERVO:1:{joint_positions[1]}\n", 3, True, "END_RESPONSE"))

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

            time.sleep(o.polling_period)
    except KeyboardInterrupt:
        print("Interrupt Caught. Exiting.")
    finally:
        o.disconnect()
        
    
if __name__ == '__main__':
    main()