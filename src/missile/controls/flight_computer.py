from missile.state import MissileState

class FlightComputer:
    def __init__(self, state: MissileState):
        self.state = state