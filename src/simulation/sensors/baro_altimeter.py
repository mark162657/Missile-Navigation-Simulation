import numpy as np

class BaroAltimeter:
    def get_baro_msl(self, true_alt: float) -> float:
        """Apply error on the true altitude (MSL). 0.5 = +- 50cm (Source: BMP388)"""
        return float(true_alt) + np.random.normal(0, 0.5)
