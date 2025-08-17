""" This modules contains WRO Future Engineer 2025 mat functions and constants."""
import math

#based on color.py from buildhat
def mat_color( r, g, b)-> str:
    """Return the color name from RGB

    :param r: Red
    :param g: Green
    :param b: Blue
    :return: Name of the color as a string
    :rtype: str
    """
    table = [("black", (0, 0, 0)),
                ("orange", (121, 60, 60)), #r to 145 , g to 85, b to 82 based on test color.
                ("blue", (50, 50, 120)),  #changed blue to 77 based on test color.
                ("white", (138, 152, 165)),
              #  ("line", (109, 120, 128))
                ]
    near = ""
    euc = math.inf
    for itm in table:
        cur = math.sqrt((r - itm[1][0])**2 + (g - itm[1][1])**2 + (b - itm[1][2])**2)
        if cur < euc:
            near = itm[0]
            euc = cur
    return near
