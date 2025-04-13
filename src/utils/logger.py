import logging

# ANSI color codes
COLORS = {
    logging.DEBUG:    "\033[94m",  # Blue
    logging.INFO:     "\033[92m",  # Green
    logging.WARNING:  "\033[93m",  # Yellow
    logging.ERROR:    "\033[91m",  # Red
    logging.CRITICAL: "\033[95m",  # Magenta
}
RESET_COLOR = "\033[0m"

class ColorLogger(logging.Formatter):
    def format(self, record):
        color = COLORS.get(record.levelno, RESET_COLOR)
        message = super().format(record)
        return f"{color}{message}{RESET_COLOR}"
        

def get_logger(name=__name__):
    logger = logging.getLogger(name)
    ch = logging.StreamHandler()
    ch.setLevel(logging.DEBUG)
    formatter = ColorLogger("%(levelname)s - %(name)s - %(message)s")
    ch.setFormatter(formatter)
    logger.addHandler(ch)
    
    return logger