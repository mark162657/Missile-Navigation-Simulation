"""missile.controls -- the control tier: flight computer, autopilot, inner loops.

    flight_computer.py : top-level orchestration of a control cycle.
    autopilot.py       : outer-loop attitude/acceleration commands.
    guidance_law.py    : lateral-acceleration guidance laws.
    pid_controller.py  : reusable PID inner loop.
    control_input.py   : the ControlInput command struct handed to physics.
"""
