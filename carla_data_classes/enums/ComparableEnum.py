from enum import Enum


class ComparableEnum(Enum):
    """
    This class allows all inheriting Enum classes to be compared by name
    """

    def __eq__(self, other):
        return self.name == other.name
