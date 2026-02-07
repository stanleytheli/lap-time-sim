import numpy as np
import pandas as pd
from utils import gen_linted_func

# basically just an information container class for all physical variables
class Car:


    _INFTY = 999999 # infinity value for infinite acceleration

    def __init__(self, m, mu_x, mu_y, g = 9.8, 
                sim_engine = False, 
                drive_ratio = None, 
                r_wheel = None, 
                torque_value = None,
                torque_csv = None):
        """
        Parameters:
        m: kilograms. mass of car
        mu_x: unitless. forward/back friction coef
        mu_y: unitless. left/right friction coef
        g: m/s^2. gravity, optional

        sim_engine: false by default, if true, expects drive_ratio, r_wheel, and ONE of torque_value XOR torque_csv.
        drive_ratio: unitless. omega_motor / omega_wheel
        r_wheel: meters. radius of wheel

        torque_value: const torque to use if you dont have a torque curve. MUTUALLY EXCLUSIVE WITH TORQUE CSV
        torque_csv: path to csv with motor torque curve data
        """
        
        self.m = m
        self.mu_x = mu_x
        self.mu_y = mu_y
        self.g = g

        self.sim_engine = sim_engine
        if sim_engine:
            # get torque data
            if torque_value:
                self.torque = lambda omega: torque_value

            else:
                df = pd.read_csv(torque_csv)

                rpm = df.iloc[:, 0].values
                omega = rpm * 2 * np.pi / 60  # convert rpm to rad/s
                
                torque_values = df.iloc[:, 1].values
                self.torque = gen_linted_func(omega, torque_values)
        

        def a_max(v):
            """max forward/backward engine acceleration as a function of speed.
            does NOT consider friction"""
            if self.sim_engine:
                omega_motor = drive_ratio * v / r_wheel
                torque = self.torque(omega_motor)
                return torque / (m * r_wheel)
            else:
                # just return infinity if unbounded by engine
                return self._INFTY
        
        self.a_max = a_max