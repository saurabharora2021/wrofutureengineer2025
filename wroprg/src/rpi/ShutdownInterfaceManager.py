from typing import List
from base.ShutdownInterface import ShutdownInterface
import logging


class ShutdownInterfaceManager:

    logger: logging.Logger = logging.getLogger(__name__)

    def __init__(self) -> None:
        """
        Initialize the ShutdownInterfaceManager with an empty list of interfaces.
        """
        self.interfaces: List[ShutdownInterface] = []

    def add_interface(self, interface: ShutdownInterface) -> None:
        """
        Add a ShutdownInterface instance to the manager.
        :param interface: An instance of a class implementing ShutdownInterface.
        """
        if interface not in self.interfaces:
            self.interfaces.append(interface)

    def shutdown_all(self) -> None:
        """
        Call the shutdown method on all registered interfaces.
        """

        self.logger.info("Shutting down all interfaces...")
        for interface in reversed(self.interfaces):
            try:
                interface.shutdown()
            except Exception as e:
                self.logger.error(f"Error shutting down {interface}: {e}")
