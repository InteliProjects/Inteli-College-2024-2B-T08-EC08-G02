import rclpy
import logging
from threading import Lock

class RclpyManager:
    _instance = None
    _lock = Lock()

    def __new__(cls, *args, **kwargs):
        with cls._lock:
            if not cls._instance:
                cls._instance = super().__new__(cls, *args, **kwargs)
        return cls._instance

    def __init__(self):
        if not hasattr(self, '_initialized'):
            self._initialized = False
            self.logger = logging.getLogger(self.__class__.__name__)

    def init(self):
        """Initialize rclpy if not already initialized."""
        if self._initialized:
            self.logger.info("rclpy is already initialized. Skipping initialization.")
            return

        with RclpyManager._lock:  # Ensure thread-safe initialization
            if not self._initialized:  # Double-check initialization state
                rclpy.init()  # Initialize rclpy once
                self._initialized = True
                self.logger.info("rclpy initialized.")

    def shutdown(self):
        """Shutdown rclpy if it is initialized."""
        if self._initialized and rclpy.ok():
            rclpy.shutdown()
            self._initialized = False
            self.logger.info("rclpy shutdown.")
