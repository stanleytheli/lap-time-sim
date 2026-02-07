# basically just an information container class for all physical variables
class Car:
    def __init__(self, m, mu_x, mu_y, g = 9.8):
        """
        Parameters:
        m: kilograms. mass of car
        mu_x: unitless. forward/back friction coef
        mu_y: unitless. left/right friction coef
        g: m/s^2. gravity, optional."""
        self.m = m
        self.mu_x = mu_x
        self.mu_y = mu_y
        self.g = g