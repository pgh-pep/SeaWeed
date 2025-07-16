from enum import IntEnum


class JoyEnum(IntEnum):
    # Logitech Gamepad F310

    # Buttons
    A = 0  # Cross/A
    B = 1  # Circle/B
    X = 2  # Square/X
    Y = 3  # Triangle/Y
    LB = 4  # L1/Left Bumper
    RB = 5  # R1/Right Bumper
    BACK = 6  # Select/Back
    START = 7  # Start
    HOME = 8  # PS/Xbox
    LS = 9  # Left stick click
    RS = 10  # Right stick click

    # Axes
    LEFT_STICK_X = 0  # Left stick horizontal
    LEFT_STICK_Y = 1  # Left stick vertical
    LEFT_TRIGGER = 2  # L2/Left trigger
    RIGHT_STICK_X = 3  # Right stick horizontal
    RIGHT_STICK_Y = 4  # Right stick vertical
    RIGHT_TRIGGER = 5  # R2/Right trigger
    DPAD_X = 6  # D-pad horizontal
    DPAD_Y = 7  # D-pad vertical
