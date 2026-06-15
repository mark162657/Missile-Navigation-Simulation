import numpy as np

from missile.state import MissileState

class BaroAltimeter():
    def __init__(self):
        self.state = MissileState()
    
    def get_baro_msl(self):
        """Apply error on the true altitude (MSL). 0.5 = +- 50cm (Source: BMP388)"""
        return self.state.true_alt + np.random.normal(0, 0.5)
    