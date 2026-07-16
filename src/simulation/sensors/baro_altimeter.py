import numpy as np

class BaroAltimeter:
    def get_baro_msl(self, true_alt: float) -> float:
        """
        Apply error on the true altitude (MSL). 0.5 = +- 50cm (Source: BMP388)

        Args:
            true_alt: the true altitude of the missile.

        Return:
            The MSL altitude that are added error to.
        """
        return float(true_alt) + np.random.normal(0, 0.5)
