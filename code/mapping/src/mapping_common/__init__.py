from abc import ABC, abstractmethod


class ROSMessageConvertible(ABC):

    @abstractmethod
    @staticmethod
    def from_ros_msg(msg):
        pass

    @abstractmethod
    def to_ros_msg(msg):
        pass
