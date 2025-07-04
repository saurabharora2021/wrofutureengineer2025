""" This module defines the Shutdown Interface"""
from abc import ABC, abstractmethod

# Define Interface for shutdown control


class ShutdownInterface(ABC):
    """ Interface for Define Shutdown callback."""
    @abstractmethod
    def shutdown(self) -> None:
        """Method to perform shutdown operations."""
