# Central algorithm:
# Solve for speed at constraint points (apexes, start, end)
# start, end: v = 0
# apexes are local minimums of speed so all acceleration goes toward them (a_y = mu_y * g)
# a = mu_y * g = v^2/r ==> sqrt(mu_y * g * r)

# NOTE: this changes under air drag!
# air drag: have to allocate some friction to countering air drag
# this probably changes the constraint points, but, uhh, let's assume that's negligible
# at constraint points: a_x = k v^2 (k is total drag coef)
# solving again, we obtain v = sqrt(mu_y g r) / 4throot(1 + (mu_y r k / mu_x)^2)

# expand out from each constraint point
# v(x + dx) = sqrt(v_0^2 + 2 a_x dx)
# to find a_x: use traction ellipse assumption and a_y = v_0^2/r
# (a_x/mu_x g)^2 + (a_y/mu_y g)^2 = 1 (operating at 'pareto efficiency' of acceleration)
# ==> a_x = mu_x g sqrt(1 - (v_0^2/r mu_y g)^2)

# to account for engine, the thing above is actually a_x_friction
# REAL a_x = min(a_x_engine, a_x_friction)

# after expanding, find minimum at each point, then calculate time as sum(dx/v)

# frictional coefficients
# x: forward/backward, y: left/right
# mu_x = 1, mu_y = 1

import numpy as np

class Constraint:
    def __init__(self, track, i, rest=False):
        self.track = track
        self.i = i
        self.rest = rest

    def register_car(self, car):
        self.car = car

        r = self.track.r
        i = self.i
        self.v = np.ndarray(shape=r.shape)

        self.a_x_applied = np.ndarray(shape=r.shape) # just for data keeping purposes

        if self.rest:
            self.v[i] = 0
        else:

            if self.car.total_drag_coef is not None:
                falloff = np.power(1 + (car.mu_y * r[i] * car.total_drag_coef / car.mu_x) ** 2, 0.25)
                self.v[i] = np.sqrt(car.mu_y * car.g * r[i]) / falloff
                self.a_x_applied[i] = self.car.total_drag_coef * (self.v[i] ** 2)
            else:
                self.v[i] = np.sqrt(car.mu_y * car.g * r[i])
                self.a_x_applied[i] = 0

    def full_solve(self):
        v, r = self.v, self.track.r
        N = len(v)
        dx = self.track.dx

        mu_x, mu_y = self.car.mu_x, self.car.mu_y
        g = self.car.g

        # accelerate
        for j in range(self.i + 1, N):
            self.v[j], self.a_x_applied[j] = self.step(self.v[j - 1], r[j], mu_x, mu_y, g, dx, forward=True)
        
        # decelerate
        for j in range(self.i - 1, -1, -1):
            self.v[j], self.a_x_applied[j] = self.step(self.v[j + 1], r[j], mu_x, mu_y, g, dx, forward=False)
        
        return self.v, self.a_x_applied
    
    def step(self, v_0, r, mu_x, mu_y, g, dx, forward=True):
        # numerical stability check
        if (r * mu_y * g) < (v_0 ** 2):
            a_x_applied = self.car.total_drag_coef * (v_0 ** 2) if self.car.total_drag_coef else 0
            return v_0, a_x_applied

        a_x_friction = mu_x * g * np.sqrt(1 - (v_0 ** 2 / (r * mu_y * g)) ** 2)
        a_x_engine = self.car.a_max(v_0)
        a_x_applied = min(a_x_engine, a_x_friction)

        # air resistance
        total_drag_coef = self.car.total_drag_coef

        a_x = a_x_applied
        if total_drag_coef is not None:
            a_drag = total_drag_coef * (v_0 ** 2)

            if forward:
                # forward: drag against you
                a_x -= a_drag
                if a_x < 0:
                    a_x = 0
            else:
                # backward: drag helping you
                a_x += a_drag

        if not forward:
            # we'll make a_x_applied signed instead of a magnitude
            a_x_applied = -a_x_applied
        
        return np.sqrt(v_0 ** 2 + 2 * a_x * dx), a_x_applied
