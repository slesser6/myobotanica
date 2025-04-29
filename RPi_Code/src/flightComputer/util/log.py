import logging, sys
def init(level=logging.DEBUG):
    logging.basicConfig(
        level=level,
        format="%(asctime)s %(name)-12s %(levelname)-7s %(message)s",
        stream=sys.stdout,
    )
