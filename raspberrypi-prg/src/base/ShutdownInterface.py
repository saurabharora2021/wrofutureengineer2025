from abc import ABC, abstractmethod

# Define Interface for shutdown control

class ShutdownInterface(ABC):
    @abstractmethod
    def shutdown(self):
        """Method to perform shutdown operations."""
        pass 