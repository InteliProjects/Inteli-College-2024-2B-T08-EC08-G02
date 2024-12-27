import logging
from logging.handlers import RotatingFileHandler
import os

log_directory = "app"
if not os.path.exists(log_directory):
    os.makedirs(log_directory)

log_file = os.path.join(log_directory, "app.log")

handler = RotatingFileHandler(log_file, maxBytes=10 * 1024 * 1024, backupCount=5)

formatter = logging.Formatter(
    "%(asctime)s - %(name)s - %(levelname)s - %(message)s "
    "[in %(pathname)s:%(lineno)d %(funcName)s]"
)
handler.setFormatter(formatter)

logger = logging.getLogger("app_logger")
logger.setLevel(logging.INFO)
logger.addHandler(handler)

def get_logger():
    return logger
