import numpy as np

class DiffDriveFour:
    def __init__(self,closed_radius,extended_radius,wheel_dist,number_arcs):
        self.rad = closed_radius
        self.ext_rad = extended_radius
        self.dist = wheel_dist
        self.A = np.array([[1, -1], [-1, -1], [1, -1], [-1, -1]])
        self.n_arc = number_arcs

    def diff_drive_four(self, v, w, h):
        # extension as ratio of radius
        e_tar = np.ones(4) * (h / self.rad - 1)

        # differential drive math for 4 wheels
        A = (1 / self.rad) * np.matmul(self.A, [np.array([[1, 0], [0, self.dist / 2]])])
        r_dot = np.matmul(A, np.array([v, w]))

        return r_dot, e_tar
