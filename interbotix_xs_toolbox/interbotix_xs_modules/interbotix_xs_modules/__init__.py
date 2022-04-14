"""This library contains modules used to control and command Interbotix X-Series Robots,
along with useful helper methods and objects specific to them.
"""

from . import xs_robot
from . import xs_launch

__all__ = [
    'xs_robot',
    'xs_launch',
]
