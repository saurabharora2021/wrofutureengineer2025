from base.ShutdownInterface import ShutdownInterface
import logging


class ShutdownInterfaceManager:

    logger = logging.getLogger(__name__)

    def __init__(self):
        """
        Initialize the ShutdownInterfaceManager with an empty list of interfaces.
        """
        self.interfaces = []

    def add_interface(self, interface: ShutdownInterface):
        """
        Add a ShutdownInterface instance to the manager.
        :param interface: An instance of a class implementing ShutdownInterface.
        """
        if interface not in self.interfaces:
            self.interfaces.append(interface)

    def shutdown_all(self):
        """
        Call the shutdown method on all registered interfaces.
        """

        self.logger.info("Shutting down all interfaces...")
        for interface in reversed(self.interfaces):
            try:
                interface.shutdown()
            except Exception as e:
                self.logger.error(f"Error shutting down {interface}: {e}")
