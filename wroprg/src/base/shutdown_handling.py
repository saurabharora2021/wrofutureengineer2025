""" This module defines the Shutdown Interface and Shutdown Interface Manager."""
from abc import ABC, abstractmethod
import logging
from typing import List
# Define Interface for shutdown control

logger = logging.getLogger(__name__)
class ShutdownInterface(ABC):
    """ Interface for Define Shutdown callback."""
    @abstractmethod
    def shutdown(self) -> None:
        """Method to perform shutdown operations."""


class ShutdownInterfaceManager:
    """ This class implements the Shutdown Manager, where all classes
    would use to register callback."""

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

        logger.info("Shutting down all interfaces...")
        for interface in reversed(self.interfaces):
            try:
                interface.shutdown()
            except Exception as e: # pylint: disable=broad-except
                logger.error("Error shutting down %s",e)
