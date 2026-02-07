import numpy as np 

def gen_linted_func(x_table, y_table, left_default=None, right_default=None):
    """
    get linearly interpolated function g satisfying g(x_i) = y_i for all x_i, y_i in x_table, y_table

    Parameters:
    x_table : an increasing array
    y_table : f(x) for x in x_table. g approximates f
    left_default : value to return for x < x_table[0], default: y_table[0]
    right_default : value to return for x > x_table[-1], default: y_table[-1]
    """
    if left_default is None:
        left_default = y_table[0]
    if right_default is None:
        right_default = y_table[-1]

    def interpolated_func(x):
        if x <= x_table[0]:
            return left_default
        if x >= x_table[-1]:
            return right_default

        for i, x_i in enumerate(x_table):
            if x_i > x:
                x_left = x_table[i - 1]
                x_right = x_i
                y_left = y_table[i - 1]
                y_right = y_table[i]
                return y_left + (y_right - y_left) * (x - x_left) / (x_right - x_left)
        
        return right_default
    return interpolated_func
