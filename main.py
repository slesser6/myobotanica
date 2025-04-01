from src.orchestrator import Orchestrator

def main():

    o = Orchestrator()
    o.connect()

    try:
        while(1):
            classification, position = o.poll_sensors()
            joint_positions = o.run_calculations(position)
            o.send_output(classification, joint_positions)

    finally:
        o.disconnect()
        
    
if __name__ == '__main__':
    main()