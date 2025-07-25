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
                ("orange", (145, 85, 82)), #r to 145 , g to 85, b to 82 based on test color.
                ("blue", (0, 51, 77)),  #changed blue to 77 based on test color.
                ("white", (255, 255, 255)),
                ("line", (179, 179, 179))
                ]
    near = ""
    euc = math.inf
    for itm in table:
        cur = math.sqrt((r - itm[1][0])**2 + (g - itm[1][1])**2 + (b - itm[1][2])**2)
        if cur < euc:
            near = itm[0]
            euc = cur
    return near
