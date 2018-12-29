using FlightMechanics
using FlightMechanics.Simulator.Aircrafts
using FlightMechanics.Simulator.Models

@static if VERSION < v"0.7.0-DEV.2005"
    using Base.Test
else
    using Test
end



@testset "aerodynamics utils" begin
    ac = F16()

    @test typeof(ac)<:Aircraft
    @test get_name(ac) == "F16"
    @test typeof(ac.aerodynamics) == F16Aerodynamics

    import FlightMechanics.Simulator.Aircrafts: get_interp_idx
    k, l, da = get_interp_idx(2.5, 0.2, -1, 8)
    @test (k, l) == (3, 4)
    @test isapprox(da, 0.5)

    k, l, da = get_interp_idx(1, 0.2, -1, 8)
    @test (k, l) == (3, 4)
    @test isapprox(da, 0.2)

    k, l, da = get_interp_idx(-1, 0.2, -1, 8)
    @test (k, l) == (2, 3)
    @test isapprox(da, 0.8)

    k, l, da = get_interp_idx(0.0, 0.2, -1, 8)
    @test (k, l) == (3, 3)
    @test isapprox(da, 0.0)

    k, l, da = get_interp_idx(-10.0, 0.2, -1, 8)
    @test (k, l) == (2, 1)
    @test isapprox(da, -1.0)

    k, l, da = get_interp_idx(45.0, 0.2, -1, 8)
    @test (k, l) == (11, 12)
    @test isapprox(da, 1.0)

    import FlightMechanics.Simulator.Aircrafts: interp1d, CXq_data
    r = interp1d(0.0, 0.2, -1, 8, CXq_data)
    @test isapprox(r, 0.308)

    r = interp1d(45, 0.2, -1, 8, CXq_data)
    @test isapprox(r, 1.210)

    r = interp1d(-10, 0.2, -1, 8, CXq_data)
    @test isapprox(r, -0.267)

    import FlightMechanics.Simulator.Aircrafts: interp2d, CX_data
    r = interp2d(0.0, 0.0, 0.2, 1/12., -1, -1, 8, 1, CX_data)
    @test isapprox(r, -0.021)

    r = interp2d(45.0, 24.0, 0.2, 1/12., -1, -1, 8, 1, CX_data)
    @test isapprox(r, 0.040)

    r = interp2d(-10.0, -24.0, 0.2, 1/12., -1, -1, 8, 1, CX_data)
    @test isapprox(r, -0.099)

    import FlightMechanics.Simulator.Aircrafts: interp2d2, Cl_data, Cn_data
    r = interp2d2(0.0, 0.0, .2, .2, -1, 1, 8, 5, Cl_data)
    @test isapprox(r, 0.0)

    r = interp2d2(-10.0, 0.0, .2, .2, -1, 1, 8, 5, Cl_data)
    @test isapprox(r, 0.0)

    r = interp2d2(45.0, 30.0, .2, .2, -1, 1, 8, 5, Cl_data)
    @test isapprox(r, -0.076)

    r = interp2d2(0.0, 0.0, .2, .2, -1, 1, 8, 5, Cn_data)
    @test isapprox(r, 0.0)

    r = interp2d2(-10.0, 0.0, .2, .2, -1, 1, 8, 5, Cn_data)
    @test isapprox(r, 0.0)

    r = interp2d2(45.0, 30.0, .2, .2, -1, 1, 8, 5, Cn_data)
    @test isapprox(r, -0.001)

end


@testset "model test case" begin
    #  Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control
    #  and simulation: dynamics, controls design, and autonomous systems. John Wiley
    #  & Sons. (page 185 table 3.5-2)

    # INPUTS
    # U(1) = THTL =   0.9  [-]
    # U(2) = ELEV =  20    [DEG]
    # U(3) = DAIL = -15    [DEG]
    # U(4) = RDR  = -20    [DEG]

    # STATE
    # IDX     name      X        UNITS        XDOT
    # 1       VT       500       [ft/s]      -75.23724
    # 2       ALPHA      0.5     [DEG]       -0.8813491
    # 3       BETA      -0.2     [DEG]       -0.4759990
    # 4       PHI       -1       [RAD]        2.505734
    # 5       THETA      1       [RAD]        0.3250820
    # 6       PSI       -1       [RAD]        2.145926
    # 7       P          0.7     [DEG/S]     12.62679
    # 8       Q         -0.8     [DEG/S]      0.9649671
    # 9       R          0.9     [DEG/S]      0.5809759
    # 10      X       1000       [FT]       342.4439
    # 11      Y        900       [FT]      -266.7707
    # 12      ALT    10000       [FT]       248.1241
    # 13      POW       90       [%]        -58.6899

    # Integration test for aircraft just making sure everything runs
    ac = F16()

    att = Attitude(-1, 1, -1)
    pos = PositionEarth(1000*FT2M, 900*FT2M, -10000*FT2M)
    u, v, w = wind2body(500*FT2M, 0, 0, 0.5, -0.2)
    state = State(pos,
                  att,
                  [u, v, w],
                  [0.7, -0.8, 0.9], #.*DEG2RAD,
                  [0., 0., 0.],
                  [0., 0., 0.]
                 )

    fcs = F16FCS()
    # set_stick_lon(fcs, 0.5+20.0/25.0)
    set_value(fcs.de, 20.0*DEG2RAD)
    # set_stick_lat(fcs, 15.0/21.5)
    set_value(fcs.da, -15.0*DEG2RAD)
    # set_pedals(fcs, 20.0/30.0)
    set_value(fcs.dr, -20*DEG2RAD)
    # set_thtl(fcs, 0.9)
    set_value(fcs.cpl, 90.0)

    env = DefaultEnvironment()
    aerostate = AeroState(state, env)
    env = calculate_environment(env, state)

    grav = env.grav

    ac = calculate_aircraft(ac, fcs, aerostate, state, grav; consume_fuel=false)
    pfm = ac.pfm
    mass_props = get_mass_props(ac)
    mass = mass_props.mass
    inertia = mass_props.inertia

    state = get_sixdof_euler_fixed_mass_state(state)

    r = six_dof_euler_fixed_mass(state, mass, inertia, pfm.forces, pfm.moments,
                                 [160.0*SLUGFT2_2_KGM2, 0.0, 0.0])

    # [RAD/S]
    ψ_dot = 2.145926
    θ_dot = 0.3250820
    ϕ_dot = 2.505734
    p_dot = 12.62679
    q_dot = 0.9649671
    r_dot = 0.5809759
    # [FT/S]
    x_dot = 342.4439
    y_dot = -266.7707
    z_dot = 248.1241

    @test isapprox(r[7], ψ_dot, atol=1e-6)  # ψ_dot [rad/s]
    @test isapprox(r[8], θ_dot, atol=1e-7)  # θ_dot [rad/s]
    @test isapprox(r[9], ϕ_dot, atol=1e-6)  # ϕ_dot [rad/s]

    @test isapprox(r[4], p_dot)   # p_dot [rad/s]
    @test isapprox(r[5], q_dot)  # q_dot [rad/s]
    @test isapprox(r[6], r_dot)  # r_dot [rad/s]

    @test isapprox(r[10]*M2FT, x_dot, atol=1e-4)   # x_dot (ft/s)
    @test isapprox(r[11]*M2FT, y_dot, atol=1e-4)  # y_dot (ft/s)
    @test isapprox(-r[12]*M2FT, z_dot, atol=1e-4)   # z_dot (ft/s)

end


@testset "trim test" begin
    # Integration test for aircraft just making sure everything runs
    ac = F16()

    @test typeof(ac.propulsion)<:Propulsion

    @test isapprox(get_wing_area(ac), 27.870912, rtol=1e-5)

    att = Attitude(1/180*pi, 0, 0)
    pos = PositionEarth(0, 0, -1000)
    state = State(pos, att, [65., 0., 3.], [0., 0., 0.], [0., 0., 0.], [0., 0., 0.])

    env = DefaultEnvironment()
    aerostate = AeroState(state, env)
    env = calculate_environment(env, state)
    grav = env.grav

    fcs = F16FCS()
    set_stick_lon(fcs, 0.0)
    set_stick_lat(fcs, 0.5)
    set_pedals(fcs, 0.5)
    set_thtl(fcs, 0.8)

    # ac = calculate_aircraft(ac, fcs, aerostate, state, grav; consume_fuel=false)

    h = 0.0 * M2FT
    psi = 0.0  # rad
    gamma = 0.0
    turn_rate = 0.0

    #  Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control
    #  and simulation: dynamics, controls design, and autonomous systems. John Wiley
    #  & Sons. (page 193 table 3.6-2)
    trim_test_data = [
    #   TAS   thtl    AOA     DE
    #   ft/s  unit    deg     deg
        130	  0.816   45.6    20.1 ;
        140   0.736   40.3   -1.36 ;
        150   0.619   34.6    0.173;
        170   0.464   27.2    0.621;
        200   0.287   19.7    0.723;
        260   0.148   11.6   -0.09 ;
        300   0.122   8.49   -0.591;
        350   0.107   5.87   -0.539;
        400   0.108   4.16   -0.591;
        440   0.113   3.19   -0.671;
        500   0.137   2.14   -0.756;
        540   0.16    1.63   -0.798;
        600   0.2     1.04   -0.846;
        640   0.23    0.742  -0.871;
        700   0.282   0.382  -0.9  ;
        800   0.378  -0.045  -0.943
        ]

    for ii=1:size(trim_test_data, 1)
        tas_fts = trim_test_data[ii, 1]
        @testset "trim tas=$tas_fts ft/s" begin

            tas =  tas_fts * FT2M

            exp_thtl = trim_test_data[ii, 2]
            exp_α    = trim_test_data[ii, 3]
            exp_de   = trim_test_data[ii, 4]

            set_stick_lon(fcs, exp_de/25.0)
            set_thtl(fcs, exp_thtl)

            # TODO: think if last trim is used as initial condition for the next
            ac, aerostate, state, env, fcs = steady_state_trim(
                ac, fcs, env, tas, pos, psi, gamma, turn_rate,
                exp_α*DEG2RAD, 0.0, true)

            @test isapprox(ac.pfm.forces, zeros(3), atol=1e-5)
            @test isapprox(ac.pfm.moments, zeros(3), atol=1e-5)
            @test isapprox(get_tas(aerostate), tas, atol=1e-5)
            @test isapprox(get_beta(aerostate), 0.0, atol=1e-5)

            # @test isapprox(fcs.cpl.value/100, exp_thtl, atol=10.0^(-length(split(string(exp_thtl), ".")[2])))
            @test isapprox(fcs.thtl.value, exp_thtl, atol=10.0^(-length(split(string(exp_thtl), ".")[2])))
            @test isapprox(aerostate.alpha*RAD2DEG, exp_α, atol=10.0^(-length(split(string(exp_α), ".")[2])))
            @test isapprox(fcs.de.value*RAD2DEG, exp_de, atol=10.0^(-length(split(string(exp_de), ".")[2])))
        end
    end

end
