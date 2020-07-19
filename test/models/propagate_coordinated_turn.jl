using FlightMechanics
using FlightMechanics.Aircrafts
using FlightMechanics.Models
using OrdinaryDiffEq


# Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control
#  and simulation: dynamics, controls design, and autonomous systems. John Wiley
#  & Sons.
# Example 3.6-5: Simulation of a Coordinated Turn (page 197)

# Choose aircraft
ac = F16()
# Choose dynamic system
dynamic_system = SixDOFEulerFixedMass()
# Choose solver
solver = RK4()
solver_options = Dict()

# Initilizations
h = 0.0 * M2FT
tas = 502 * FT2M
psi = 0.0  # rad
gamma = 0.0  # rad
turn_rate = 0.3  # rad/s

pos = EarthPosition(0, 0, -h)

# Initilize environment
env = Environment(pos, atmos = "ISAF16", wind = "NoWind", grav = "const")

# Initilize FCS
controls_0 = StickPedalsLeverControls(0.5, 0.5, 0.5, 0.8)

# Trim a/c
ac, aerostate, state, controls = steady_state_trim(
    ac,
    controls_0,
    env,
    tas,
    pos,
    psi,
    gamma,
    turn_rate,
    0.0,
    0.0,
    show_trace = false,
    )

controls_stream = convert(ControlsStream, controls)

tini = 0.0
tfin = 180
dt = 0.01

results = propagate(
    ac,
    env,
    state,
    aerostate,
    controls_stream,
    tini,
    tfin,
    dt,
    dynamic_system,
    solver;
    solver_options=solver_options
    )

@test isapprox(
    get_height(results.state[1]),
    get_height(results.state[end]),
    )

@test isapprox(
    get_euler_angles(results.state[1])[2],
    get_euler_angles(results.state[end])[2],
)

@test isapprox(
    get_euler_angles(results.state[1])[3],
    get_euler_angles(results.state[end])[3],
)

@test isapprox(
    get_tas(results.aerostate[1]),
    get_tas(results.aerostate[end]),
)

@test isapprox(
    get_alpha(results.aerostate[1]),
    get_alpha(results.aerostate[end]),
)

@test isapprox(
    get_beta(results.aerostate[1]),
    get_beta(results.aerostate[end]),
)

@test isapprox(
    results.state[1].acceleration,
    results.state[end].acceleration,
)

@test isapprox(
    results.state[1].angular_acceleration,
    results.state[end].angular_acceleration,
)
