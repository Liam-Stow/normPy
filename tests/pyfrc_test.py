'''
    This test module imports tests that come with pyfrc, and can be used
    to test basic functionality of just about any robot.
'''

from pyfrc.tests.basic import test_autonomous, test_operator_control, test_practice

__all__ = (
    "test_autonomous",
    "test_operator_control",
    "test_practice",
)
