"""Single source of truth for the robotaxi's physical parameters (sent to the browser via /api/map)."""
VEHICLE = dict(
    m0=1850.0,     # empty mass, kg (+80 kg per rider)
    c=0.527,       # aerodynamic drag coefficient 0.5*rho*Cd*A
    crr=0.012,     # rolling resistance
    Fmax=5600.0,   # traction force limit, N
    Pmax=60000.0,  # motor power, W
    ta=0.25,       # actuator lag, s
    th=0.08,       # actuator dead time, s
    Ts=0.02,       # control period used in the dead-time estimate, s
)
