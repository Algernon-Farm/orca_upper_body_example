import numpy as np


class TCurvePlanner:
    def __init__(self, q0: np.ndarray, q1: np.ndarray, vmax: np.ndarray, amax: np.ndarray) -> None:
        super().__init__()

        self._q0 = np.array(q0)
        self._q1 = np.array(q1)
        self._delta_q = self._q1 - self._q0

        # self.vmax_hat = np.min(np.abs(vmax / (self._q1 - self._q0)))
        # self.amax_hat = np.min(np.abs(amax / (self._q1 - self._q0)))

        self.vmax_hat = 1 / np.max(np.abs((self._q1 - self._q0) / vmax))
        self.amax_hat = 1 / np.max(np.abs((self._q1 - self._q0) / amax))

        self._ta = self.vmax_hat / self.amax_hat
        self._pa = 0.5 * self.amax_hat * np.power(self._ta, 2)
        self._tm = (1 - 2 * self._pa) / self.vmax_hat
        self._tf = self._tm + 2 * self._ta

        if self._tf - 2 * self._ta < 0:
            self._ta = np.sqrt(1 / self.amax_hat)
            self._tf = 2 * self._ta

    @property
    def tf(self) -> float:
        return self._tf

    def get_tf(self) -> float:
        return self._tf

    def interpolate(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        p = 0.0
        v = 0.0
        a = 0.0
        if t <= 0.0:
            p = 0.0
            v = 0.0
            a = 0.0
        elif t <= self._ta:
            p = 0.5 * self.amax_hat * np.power(t, 2)
            v = self.amax_hat * t
            a = self.amax_hat
        elif t < self.tf - self._ta:
            p = 0.5 * self.amax_hat * np.power(self._ta, 2) + self.vmax_hat * (t - self._ta)
            v = self.vmax_hat
            a = 0.0
        elif t <= self._tf:
            p = 1 - 0.5 * self.amax_hat * np.power(self._tf - t, 2)
            v = self.amax_hat * (self._tf - t)
            a = -self.amax_hat
        else:
            p = 1
            v = 0
            a = 0
        return self._q0 + p * self._delta_q, v * self._delta_q, a * self._delta_q


def interpolate_t_curve(start_q: np.ndarray, end_q: np.ndarray, num: int) -> tuple:
    """T型插值曲线"""
    planner = TCurvePlanner(start_q, end_q, [1.57] * num, [3.14] * num)
    tf = planner.tf
    times = np.arange(0, np.ceil(tf), 0.001)
    p = np.zeros((times.size, start_q.size))
    v = np.zeros_like(p)
    a = np.zeros_like(p)

    for i, t in enumerate(times):
        p[i, :], v[i, :], a[i, :] = planner.interpolate(t)
    return times, p, v, a
