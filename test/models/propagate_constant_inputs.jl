using LinearAlgebra
using OrdinaryDiffEq
using FlightMechanics.Aircrafts
using FlightMechanics.Models


# Choose aircraft
ac = F16()
# Choose dynamic system
dynamic_system = SixDOFEulerFixedMass()
# Choose solver
solver = RK4()
solver_options = Dict()

# Initilizations
h = 1000.0 * M2FT
tas = 250 * KT2MS
psi = 0.0  # rad
gamma = 0.0
turn_rate = 0.0

pos = EarthPosition(0, 0, -h)

# Initilize environment
env = Environment(pos, atmos = "ISAF16", wind = "NoWind", grav = "const")

att = Attitude(-1, 0.1, 0.0)

# Initilize FCS
controls_0 = StickPedalsLeverControls(0.0, 0.5, 0.5, 0.8)

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

times_to_test = [0.1, 1.0, 10.0, 100.0]  # seconds

@testset "Simulation time $t s" for t=times_to_test
    tini = 0.0
    tfin = t
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
        results.state[1].attitude,
        results.state[end].attitude,
    )

    @test isapprox(
        results.state[1].velocity,
        results.state[end].velocity,
    )

    @test isapprox(
        results.state[1].angular_velocity,
        results.state[end].angular_velocity,
        atol=1e-11
    )

    @test isapprox(
        results.state[1].acceleration,
        results.state[end].acceleration,
        atol=1e-10
    )

    @test isapprox(
        results.state[1].angular_acceleration,
        results.state[end].angular_acceleration,
        atol=1e-12
    )

    @test isapprox(
        get_height(results.state[1]),
        get_height(results.state[end]),
        )

    @test isapprox(
        norm(get_xyz_earth(results.state[1])[1:2] - get_xyz_earth(results.state[end])[1:2]),
        tas * (tfin - tini),
        )
end
