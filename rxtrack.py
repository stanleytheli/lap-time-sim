import numpy as np
from constraint import Constraint

class rxTrack:
    """Track parametrized by radius of curvature as a function of distance
    along the track, r(x). Assuming left-right symmetry guarantees that 
     we can abstract from the actual form of the track. """
    
    _INFTY = 999999 # infinity value used for straight sections

    def __init__(self, dx, r = None): # r = None because python default args don't work w/ objects
        """
        Parameters:
        dx: meters. resolution of the track model 
        r: meters. radius of curvature, numpy array, optional. r[i] = radius at (i * dx) along track
        """
        self.dx = dx
        self.r = r if r else np.ndarray(shape=(0,)) # empty np.ndarray if r empty
        self.constraints : list[Constraint] = []
    
    def add_circular_section(self, r, x):
        """
        Parameters:
        r: meters. radius
        x: meters. length of section 
        """
        N = int(x / self.dx)
        addon = np.ones(shape=(N,), dtype=np.float64) * r

        self.constraints.append(Constraint(self, len(self.r)))
        self.r = np.append(self.r, addon)

    def add_straight_section(self, x):
        """
        Parameters:
        x: meters. length of section
        """   
        N = int(x / self.dx)
        addon = np.ones(shape=(N,), dtype=np.float64) * self._INFTY
        self.r = np.append(self.r, addon)

    def add_constraint(self, constraint : Constraint):
        self.constraints.append(constraint)
    
    def add_start_end_constraints(self):
        self.constraints.append(Constraint(self, 0, rest=True))
        self.constraints.append(Constraint(self, len(self.r) - 1, rest=True))

    def compute_v(self, car):
        self.vs = np.ndarray(shape=(len(self.constraints), len(self.r)))
        self.a_x_applieds = np.ndarray(shape=(len(self.constraints), len(self.r)))

        for i, constraint in enumerate(self.constraints):
            constraint.register_car(car)
            self.vs[i], self.a_x_applieds[i] = constraint.full_solve()

        idxs = np.argmin(self.vs, axis=0)
        self.v = np.min(self.vs, axis=0)
        self.a_x_applied = [a_x_app[idx] for a_x_app, idx in zip(self.a_x_applieds.T, idxs)]

        return {"v(i)" : self.v, "vs(i)" : self.vs, "a_x_applied(i)" : self.a_x_applied}
    
    def solve(self, car):
        output = self.compute_v(car)
        v = output["v(i)"]
        a_x_applied = output["a_x_applied(i)"]
        N = len(v)
        dx = self.dx
        r = self.r
        m = car.m
    
        output["x_axis"] = np.arange(N) * dx

        # attach radius
        output["r(i)"] = self.r        

        # compute times
        times = []
        lap_time = 0
        for i in range(0, len(v)):

            if v[i] == 0: # for points where v=0, estimate lap time using nearby points
                if i + 1 < N and v[i + 1] != 0:
                    # factor of 2 from constant acceleration assumption
                    lap_time += 2 * dx / v[i + 1] 
                elif i - 1 >= 0 and v[i - 1] != 0:
                    lap_time += 2 * dx / v[i - 1]
                times.append(lap_time)
                continue

            # otherwise, just use the simple calculation
            lap_time += dx / v[i]
            times.append(lap_time)

        output["t_axis"] = times
        output["lap_time"] = lap_time

        # compute acceleration (net, rather than applied)
        a_x = []
        a_y = []
        for i in range(0, len(v)):
            if i == 0:
                v_0 = 0
            else:
                v_0 = v[i - 1]
            v_1 = v[i]
            # just reverse calculate the acceleration
            a_x.append((v_1**2 - v_0**2) / (2 * dx)) 
            a_y.append(v[i]**2 / self.r[i])
        output["a_x(i)"] = a_x
        output["a_y(i)"] = a_y

        # compute energy usage
        force = []
        energy = []
        total_energy = 0
        power = []
        for i in range(0, len(v)):
            # F_applied = m a_x_applied
            # dE = F_applied * dx
            # dP = dE / dt = F_applied * v

            force.append(m * abs(a_x_applied[i]))
            power.append(m * abs(a_x_applied[i]) * v[i])

            total_energy += m * abs(a_x_applied[i]) * dx
            energy.append(total_energy)

        output["F(i)"] = force
        output["E(i)"] = energy
        output["P(i)"] = power
        output["total_energy"] = total_energy

        return output

class ExampleTracks:
    j_track = rxTrack(0.5)
    j_track.add_straight_section(500)
    j_track.add_circular_section(50, np.pi * 50)
    j_track.add_start_end_constraints()

    straight_track = rxTrack(0.5)
    straight_track.add_straight_section(1000)
    straight_track.add_start_end_constraints()

    circular_track = rxTrack(0.5)
    circular_track.add_circular_section(200, 2 * np.pi * 200)
    circular_track.add_start_end_constraints()

    pill_track = rxTrack(0.5)
    pill_track.add_straight_section(250)
    pill_track.add_circular_section(50, np.pi * 50)
    pill_track.add_straight_section(250)
    pill_track.add_circular_section(50, np.pi * 50)
    pill_track.add_start_end_constraints()
