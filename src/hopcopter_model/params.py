"""Stance-phase model parameters.

The stance dynamics depend only on the per-mass quantities k/m and f_c/m,
so :class:`StanceParams` stores those directly (SI units).
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class StanceParams:
    """Parameters of the passive telescopic leg.

    Attributes:
        k_over_m: spring stiffness per mass, k/m (s^-2).
        f_c_over_m: Coulomb friction force per mass, f_c/m (m/s^2).
        l_p: preload extension of the elastic element (m).
        l0: leg rest length from foot tip to body centre of mass (m).
    """

    k_over_m: float = 5.00e3
    f_c_over_m: float = 12.7
    l_p: float = 0.0197
    l0: float = 0.22

    @classmethod
    def from_mass_params(cls, k, m, f_c, l0, l_p):
        """Build from physical spring/friction values (k, f_c in N/m and N)."""
        return cls(k_over_m=k / m, f_c_over_m=f_c / m, l_p=l_p, l0=l0)

    @property
    def omega(self):
        """Natural frequency of the axial spring-mass system, sqrt(k/m)."""
        return self.k_over_m ** 0.5

    @property
    def c_down(self):
        """Equilibrium leg length during compression (Coulomb force adds to preload)."""
        return self.l0 + self.l_p + self.f_c_over_m / self.k_over_m

    @property
    def c_up(self):
        """Equilibrium leg length during extension (Coulomb force subtracts from preload)."""
        return self.l0 + self.l_p - self.f_c_over_m / self.k_over_m


#: Parameters identified in the paper (k/m = 5.00e3 s^-2, f_c/m = 12.7 m/s^2).
PAPER_PARAMS = StanceParams(k_over_m=5.00e3, f_c_over_m=12.7, l_p=0.0197, l0=0.22)

#: Parameters used inside the authors' MATLAB code (k = 170.0615 N/m, m = 34 g,
#: f_h = 0.4453 N), kept for cross-checking against the Zenodo scripts.
AUTHORS_PARAMS = StanceParams.from_mass_params(
    k=170.0615, m=34 / 1000, f_c=0.4453, l0=0.22, l_p=0.0197
)
