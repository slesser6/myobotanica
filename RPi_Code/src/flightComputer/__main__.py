from flightComputer.util.log import init
from flightComputer.routing.dispatcher import Dispatcher
from flightComputer.fc import FC   # ensures connection on import

def main():
    init()
    FC.connect()
    Dispatcher().run()

if __name__ == "__main__":
    main()
