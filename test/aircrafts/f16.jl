using Test
using LinearAlgebra
using FlightMechanics
using FlightMechanics.Aircrafts
using FlightMechanics.Models


@testset "aerodynamics utils" begin
    ac = F16()

    @test typeof(ac)<:Aircraft
    @test get_name(ac) == "F16"
    @test typeof(ac.aerodynamics) == F16Aerodynamics

    import FlightMechanics.Aircrafts: get_interp_idx
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

    import FlightMechanics.Aircrafts: interp1d, CXq_data
    r = interp1d(0.0, 0.2, -1, 8, CXq_data)
    @test isapprox(r, 0.308)

    r = interp1d(45, 0.2, -1, 8, CXq_data)
    @test isapprox(r, 1.210)

    r = interp1d(-10, 0.2, -1, 8, CXq_data)
    @test isapprox(r, -0.267)

    import FlightMechanics.Aircrafts: interp2d, CX_data
    r = interp2d(0.0, 0.0, 0.2, 1/12., -1, -1, 8, 1, CX_data)
    @test isapprox(r, -0.021)

    r = interp2d(45.0, 24.0, 0.2, 1/12., -1, -1, 8, 1, CX_data)
    @test isapprox(r, 0.040)

    r = interp2d(-10.0, -24.0, 0.2, 1/12., -1, -1, 8, 1, CX_data)
    @test isapprox(r, -0.099)

    import FlightMechanics.Aircrafts: interp2d2, Cl_data, Cn_data
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


@testset "aerodynamics" begin
    ac = F16()

    # TEST against Morelli
    input_data = [
        # tas    α    β    p    q    r    de    da    dr    xcg
        500    0.5  -0.2  0.7  -0.8  0.9  20    -15  -20    0.35;
        500    0.5  -0.2  0.0   0.0  0.0  20    -15  -20    0.35;
        500    0.5  -0.2  0.0   0.0  0.0   0      0    0    0.35;
        500    0.0   0.0  0.0   0.0  0.0   0      0    0    0.35;
        500    0.5  -0.2 -0.8   0.0  0.0  20    -15  -20    0.35;
    ]

    # Data obatined from Morelli
    out_data = [
        -0.0666571605333333 -0.0485321333333333  -0.0196159964939023  0.0213716333333333 -0.116409952000000  0.0272818333333333;
        -0.0629333333333333 -0.0690833333333333  -0.283598396493902   0.0287903333333333 -0.163800000000000  0.0365518333333333;
        -0.0193000000000000  0.00400000000000000 -0.131598396493902   3.35999999999999e-04 -0.00860000000000000   -7.23999999999999e-04;
        -0.0210000000000000  0.0                 -0.100000000000000   0                    -0.00900000000000000   0;
        -0.0629333333333333 -0.0652865333333333 -0.283598396493902  0.0393671333333333 -0.163800000000000  0.0354574333333333;
    ]

    att = Attitude(-1, 1, -1)
    pos = EarthPosition(1000*FT2M, 900*FT2M, -10000*FT2M)
    env = Environment(pos, atmos="ISA1978", wind="NoWind", grav="const")
    fcs = get_fcs(ac)


    @testset "Case $ii" for ii=1:size(input_data, 1)
        tas, α, β, p, q, r, de, da, dr, xcg = input_data[ii, :]
        cx, cy, cz, cl, cm, cn = out_data[ii, :]

        u, v, w = wind2body(tas*FT2M, 0, 0, α*DEG2RAD, β*DEG2RAD)
        state = State(pos,
                      att,
                      [u, v, w],
                      [p, q, r],
                      [0., 0., 0.],
                      [0., 0., 0.]
                     )
        # Act directly on FCS surfaces
        set_value!(fcs.de, de*DEG2RAD)
        set_value!(fcs.da, da*DEG2RAD)
        set_value!(fcs.dr, dr*DEG2RAD)

        aerostate = AeroState(state, env)

        # Calculate aerodynamics
        aero = get_aerodynamics(ac)
        aero = calculate_aerodynamics(ac, aero, aerostate, state)

        pfm = get_body_adim_pfm(aero)

        @test isapprox(pfm.forces[1], cx)
        @test isapprox(pfm.forces[2], cy)
        @test isapprox(pfm.forces[3], cz)
        @test isapprox(pfm.moments[1], cl)
        @test isapprox(pfm.moments[2], cm)
        @test isapprox(pfm.moments[3], cn)
    end
end


@testset "aerodynamics 2" begin
    # TODO: Fix breaking tests if Morelli ac is implemented
    ac = F16()

    # TEST against Morelli
    input_data = [
        # tas    α    β    p    q    r    de    da    dr    xcg
        500    28.64788975654116  -11.459155902616466  0.7  -0.8  0.9  20    -15   20    0.35;
        500    28.64788975654116  -11.459155902616466  0.7  -0.8  0.9  20    -15  -20    0.35;
    ]

    # Results from Morelli
    out_data = [
         0.04247190703  0.297332165194  -1.661302153   0.05809649017 -0.05637730554  -0.0432532826901365;
         0.04247190703  0.182665498527  -1.661302153   0.05809649017 -0.05637730554  -0.0045883093820057;
    ]

    att = Attitude(-1, 1, -1)
    pos = EarthPosition(1000*FT2M, 900*FT2M, -10000*FT2M)
    env = Environment(pos, atmos="ISA1978", wind="NoWind", grav="const")
    fcs = get_fcs(ac)


    @testset "Case $ii" for ii=1:size(input_data, 1)
        tas, α, β, p, q, r, de, da, dr, xcg = input_data[ii, :]
        cx, cy, cz, cl, cm, cn = out_data[ii, :]

        u, v, w = wind2body(tas*FT2M, 0, 0, α*DEG2RAD, β*DEG2RAD)
        state = State(pos,
                      att,
                      [u, v, w],
                      [p, q, r],
                      [0., 0., 0.],
                      [0., 0., 0.]
                     )
        set_value!(fcs.de, de*DEG2RAD)
        set_value!(fcs.da, da*DEG2RAD)
        set_value!(fcs.dr, dr*DEG2RAD)

        aerostate = AeroState(state, env)

        # Calculate aerodynamics
        aero = get_aerodynamics(ac)
        aero = calculate_aerodynamics(ac, aero, aerostate, state)

        pfm = get_body_adim_pfm(aero)

        @test isapprox(pfm.forces[1], cx)
        @test isapprox(pfm.forces[2], cy)
        @test isapprox(pfm.forces[3], cz)
        @test_broken isapprox(pfm.moments[1], cl)  # Differences in Morelli
        @test isapprox(pfm.moments[2], cm)
        @test isapprox(pfm.moments[3], cn)
    end
end


@testset "engine" begin

    test_data = [
        # thtl (-)   alt(ft)   tas (ft/s)    thrust(N)
           0.0        10000      500       -940.891895587440;
           0.1        10000      500       4659.15413340486;
           0.2        10000      500      10259.2001623972;
           0.3        10000      500      15859.2461913895;
           0.4        10000      500      21459.2922203818;
           0.5        10000      500      27059.3382493741;
           0.6        10000      500      32659.3842783664;
           0.7        10000      500      38259.4303073587;
           0.8        10000      500      46841.5273717983;
           0.9        10000      500      62386.4689396095;
           1.0        10000      500      77931.4105074207
    ]

    ac = F16()
    eng = F16Engine()
    fcs = get_fcs(ac)
    att = Attitude(0., 0., 0.)
    pos = EarthPosition(0, 0, 0)
    env = Environment(pos, atmos="ISAF16", wind="NoWind", grav="const")


    @testset "Case $ii" for ii=1:size(test_data, 1)
        pos = EarthPosition(0, 0, -test_data[ii, 2]*FT2M)
        env = calculate_environment(env, pos)
        u, v, w = wind2body(test_data[ii, 3]*FT2M, 0., 0., 0., 0.)
        set_thtl!(fcs, test_data[ii, 1])
        state = State(pos, att, [u, v, w], [0., 0., 0.], [0., 0., 0.], [0., 0., 0.])
        aerostate = AeroState(state, env)

        eng = calculate_engine(eng, fcs, aerostate, state)

        thrust = get_pfm(eng).forces[1]

        @test isapprox(thrust, test_data[ii, 4], atol=1e-3)
    end
end


@testset "trim test - level flight" begin
    # Integration test for aircraft just making sure everything runs
    ac = F16()

    fcs = get_fcs(ac)
    controls = StickPedalsLeverControls(0.0, 0.5, 0.5, 0.8)
    set_controls!(ac, controls)

    h = 0.0 * M2FT
    psi = 0.0  # rad
    gamma = 0.0
    turn_rate = 0.0

    pos = EarthPosition(0, 0, -h)
    env = Environment(pos, atmos="ISAF16", wind="NoWind", grav="const")

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
        640   0.23    0.742  -0.871;
        800   0.378  -0.045  -0.943;
        200   0.287   19.7    0.723;
        260   0.148   11.6   -0.09 ;
        300   0.122   8.49   -0.591;
        350   0.107   5.87   -0.539;
        400   0.108   4.16   -0.591;
        440   0.113   3.19   -0.671;
        500   0.137   2.14   -0.756;
        540   0.16    1.63   -0.798;
        600   0.2     1.04   -0.846;
        700   0.282   0.382  -0.9  ;
        ]

    # Cases with small deviations in DE
    @testset "trim tas=$(trim_test_data[ii, 1]) ft/s" for ii=1:4
        tas =  trim_test_data[ii, 1] * FT2M

        exp_thtl = trim_test_data[ii, 2]
        exp_α    = trim_test_data[ii, 3]
        exp_de   = trim_test_data[ii, 4]

        # Initial conditions for trimmer
        α0 = 0.5*exp_α*DEG2RAD
        β0 = 0.
        stick_lon0 = 0.
        thtl0 = 0.5

        controls = StickPedalsLeverControls(
            stick_lon0,
            0.5,
            0.5,
            thtl0
        )

        set_controls!(ac, controls)

        ac, aerostate, state, controls = steady_state_trim(
            ac, controls, env, tas, pos, psi, gamma, turn_rate, α0, 0.0, show_trace=false
            )

        fcs = get_fcs(ac)
        @test isapprox(ac.pfm.forces, zeros(3), atol=1e-7)
        @test isapprox(ac.pfm.moments, zeros(3), atol=1e-7)
        @test isapprox(get_tas(aerostate), tas)
        @test isapprox(get_beta(aerostate), 0.0, atol=1e-13)

        @test isapprox(fcs.thtl.value, exp_thtl, atol=1e-3)
        @test isapprox(aerostate.alpha*RAD2DEG, exp_α, atol=10.0^(-length(split(string(exp_α), ".")[2])))
        @test_broken isapprox(fcs.de.value*RAD2DEG, exp_de, atol=10.0^(-length(split(string(exp_de), ".")[2])))
    end

    # Cases with small deviations in α
    @testset "trim tas=$(trim_test_data[ii, 1]) ft/s" for ii=5:5
        tas =  trim_test_data[ii, 1] * FT2M

        exp_thtl = trim_test_data[ii, 2]
        exp_α    = trim_test_data[ii, 3]
        exp_de   = trim_test_data[ii, 4]

        # Initial conditions for trimmer
        α0 = 0.5*exp_α*DEG2RAD
        β0 = 0.
        stick_lon0 = 0.
        thtl0 = 0.5

        controls = StickPedalsLeverControls(
            stick_lon0,
            0.5,
            0.5,
            thtl0
        )

        set_controls!(ac, controls)

        ac, aerostate, state, controls = steady_state_trim(
            ac, controls, env, tas, pos, psi, gamma, turn_rate, α0, 0.0, show_trace=false
            )

        fcs = get_fcs(ac)

        @test isapprox(ac.pfm.forces, zeros(3), atol=1e-7)
        @test isapprox(ac.pfm.moments, zeros(3), atol=1e-7)
        @test isapprox(get_tas(aerostate), tas)
        @test isapprox(get_beta(aerostate), 0.0, atol=1e-13)

        @test isapprox(fcs.thtl.value, exp_thtl, atol=1e-3)
        @test_broken isapprox(aerostate.alpha*RAD2DEG, exp_α, atol=10.0^(-length(split(string(exp_α), ".")[2])))
        @test isapprox(fcs.de.value*RAD2DEG, exp_de, atol=10.0^(-length(split(string(exp_de), ".")[2])))
    end

    # Cases matching the test values
    @testset "trim tas=$(trim_test_data[ii, 1]) ft/s" for ii=6:size(trim_test_data, 1)
        tas =  trim_test_data[ii, 1] * FT2M

        exp_thtl = trim_test_data[ii, 2]
        exp_α    = trim_test_data[ii, 3]
        exp_de   = trim_test_data[ii, 4]

        # Initial conditions for trimmer
        α0 = 0.5*exp_α*DEG2RAD
        β0 = 0.
        stick_lon0 = 0.
        thtl0 = 0.5

        controls = StickPedalsLeverControls(
            stick_lon0,
            0.5,
            0.5,
            thtl0
        )

        set_controls!(ac, controls)

        ac, aerostate, state, controls = steady_state_trim(
            ac, controls, env, tas, pos, psi, gamma, turn_rate, α0, 0.0, show_trace=false
            )

        fcs = get_fcs(ac)

        @test isapprox(ac.pfm.forces, zeros(3), atol=1e-7)
        @test isapprox(ac.pfm.moments, zeros(3), atol=1e-7)
        @test isapprox(get_tas(aerostate), tas)
        @test isapprox(get_beta(aerostate), 0.0, atol=1e-13)

        @test isapprox(fcs.thtl.value, exp_thtl, atol=1e-3)
        @test isapprox(aerostate.alpha*RAD2DEG, exp_α, atol=10.0^(-length(split(string(exp_α), ".")[2])))
        @test isapprox(fcs.de.value*RAD2DEG, exp_de, atol=10.0^(-length(split(string(exp_de), ".")[2])))
    end
end


@testset "trim test - coord turn" begin
    #  Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control
    #  and simulation: dynamics, controls design, and autonomous systems. John Wiley
    #  & Sons. (page 192)
    pos = EarthPosition(0, 0, 0)
    env = Environment(pos, atmos="ISAF16", wind="NoWind", grav="const")

    ac = F16()

    h = 0.0 * M2FT
    psi = 0.234077 # rad
    gamma = 0.0
    turn_rate = 0.3  # rad/s
    tas_fts = 502.0  # ft/s

    tas =  tas_fts * FT2M

    exp_α    =  2.392628e-1  # rad
    exp_β    =  5.061803e-4  # rad
    exp_ϕ    =  1.366289     # rad
    exp_θ    =  5.000808e-2  # rad
    exp_ψ    =  2.340769e-1  # rad
    exp_p    = -1.499617e-2  # rad/s
    exp_q    =  2.933811e-1  # rad/s
    exp_r    =  6.084932e-2  # rad/s
    exp_x    =  0.0  # ft
    exp_y    =  0.0  # ft
    exp_z    =  0.0  # ft

    exp_thtl =  0.8349601
    exp_de   = -1.481766     # deg
    exp_da   =  9.553108e-2  # deg
    exp_dr   = -4.118124e-1  # deg


    # Initial conditions for trimmer
    α0 = 0.5*exp_α*DEG2RAD
    β0 = 0.
    stick_lon0 = 0.0
    stick_lat0 = 0.0
    pedals0 = 0.0
    thtl0 = 0.5

    controls = StickPedalsLeverControls(
            stick_lon0,
            stick_lat0,
            pedals0,
            thtl0
        )

    set_controls!(ac, controls)

    ac, aerostate, state, controls = steady_state_trim(
        ac, controls, env, tas, pos, psi, gamma, turn_rate, α0, β0, show_trace=false
        )

    fcs = get_fcs(ac)

    @test isapprox(get_tas(aerostate), tas)
    @test isapprox(get_alpha(aerostate), exp_α, atol=5e-4)
    @test isapprox(get_beta(aerostate), exp_β, atol=5e-5)

    @test isapprox(get_body_ang_velocity(state), [exp_p, exp_q, exp_r], atol=5e-5)
    @test isapprox(get_euler_angles(state), [exp_ψ, exp_θ, exp_ϕ], atol=5e-5)

    # TODO: maybe these tolerances are too broad. Take into account that trim in
    # Stevens is only based in the number of iterations.
    @test isapprox(fcs.thtl.value, exp_thtl, atol=5e-4)
    @test isapprox(fcs.de.value*RAD2DEG, exp_de, rtol=1e-2)
    @test isapprox(fcs.da.value*RAD2DEG, exp_da, rtol=1e-2)
    @test isapprox(fcs.dr.value*RAD2DEG, exp_dr, rtol=1e-1)
end


@testset "model test case" begin
    #  Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control
    #  and simulation: dynamics, controls design, and autonomous systems. John Wiley
    #  & Sons. (page 185 table 3.5-2)

    # xcg = 0.4 * CMA
    # Note: empty CG cannot be modified as a parameter so the only option is to
    # modify the getter which has its value hardcoded
    # TODO: modify when ac structure is refactored to do this
    FlightMechanics.Aircrafts.get_empty_cg(ac::F16) = [-0.40 * get_chord(ac), 0, 0]

    # INPUTS
    # U(1) = THTL =   0.9  [-]
    # U(2) = ELEV =  20    [DEG]
    # U(3) = DAIL = -15    [DEG]
    # U(4) = RDR  = -20    [DEG]

    # STATE
    # IDX     name      X        UNITS        XDOT           XDOT MORELLI
    # 1       VT       500       [ft/s]      -75.23724       -77.57521
    # 2       ALPHA      0.5     [RAD]       -0.8813491      -0.88123
    # 3       BETA      -0.2     [RAD]       -0.4759990      -0.45276
    # 4       PHI       -1       [RAD]        2.505734        0.70000
    # 5       THETA      1       [RAD]        0.3250820       0.32508
    # 6       PSI       -1       [RAD]        2.145926        2.14593
    # 7       P          0.7     [RAD/S]     12.62679        12.91108
    # 8       Q         -0.8     [RAD/S]      0.9649671      0.97006
    # 9       R          0.9     [RAD/S]      0.5809759      -0.55450
    # 10      X       1000       [FT]       342.4439         342.44390
    # 11      Y        900       [FT]      -266.7707         -266.77068
    # 12      ALT    10000       [FT]       248.1241          248.12412
    # 13      POW       90       [%]        -58.6899         -58.6900

    # XXX: From morelli implementation
    # udot, vdot, wdot = 100.84. -206.45, -436.98  ft/s
    # thrust =    1.5912e+04  lbf
    # qbar =  219.72  psf
    # mach =  0.46436

    # Integration test for aircraft just making sure everything runs
    ac = F16()

    att = Attitude(-1, 1, -1)
    pos = EarthPosition(1000*FT2M, 900*FT2M, -10000*FT2M)
    u, v, w = wind2body(500*FT2M, 0, 0, 0.5, -0.2)
    state = State(pos,
                  att,
                  [u, v, w],
                  [0.7, -0.8, 0.9],
                  [0., 0., 0.],
                  [0., 0., 0.]
                 )

    # Set controls to obtain the following deflexions
    # fcs = get_fcs(ac)
    # set_value!(fcs.de, 20.0*DEG2RAD)
    # set_value!(fcs.da, -15.0*DEG2RAD)
    # set_value!(fcs.dr, -20*DEG2RAD)
    # set_value!(fcs.cpl, 90.0)
    controls = StickPedalsLeverControls(
        0.5 * (1 + 20 / 25),  # According to DE_MAX
        0.5 * (1 - 15 / 20),  # According to DA_MAX
        0.5 * (1 - 20 / 30),  # According to DR_MAX
        (90.0 + 117.38) / 217.38  # According to tgear
    )

    env = Environment(pos, atmos="ISAF16", wind="NoWind", grav="const")
    aerostate = AeroState(state, env)
    grav = get_gravity(env)

    ac = calculate_aircraft(ac, controls, aerostate, state, grav; consume_fuel=false)
    pfm = ac.pfm
    mass_props = get_mass_props(ac)
    mass = mass_props.mass
    inertia = mass_props.inertia

    six_dof_euler_fixed_mass_ds = convert(SixDOFEulerFixedMass, state)
    x = get_x(six_dof_euler_fixed_mass_ds)
    f = get_state_equation(six_dof_euler_fixed_mass_ds)
    # Evaluate x_dot given x, u, parameters
    r = f(x, mass, inertia, pfm.forces, pfm.moments, [160.0*SLUGFT2_2_KGM2, 0.0, 0.0])
    u_dot, v_dot, w_dot, p_dot, q_dot, r_dot, ψ_dot, θ_dot, ϕ_dot, x_dot, y_dot, z_dot = r 

    # u, v, w are used in this test, so they need to be transformed to VT, α, β
    VT = 500 * FT2M
    VT_dot_est, α_dot_est, β_dot_est = tas_α_β_dot_from_uvw_dot(u, v, w, u_dot, v_dot, w_dot)

    @testset "Stevens values" begin
        # XDOT Stevens
        # [RAD/S]
        α_dot_exp = -0.8813491
        β_dot_exp = -0.4759990
        ψ_dot_exp = 2.145926
        θ_dot_exp = 0.3250820
        ϕ_dot_exp = 2.505734
        p_dot_exp = 12.62679
        q_dot_exp = 0.9649671
        r_dot_exp = 0.5809759
        # [FT/S]
        VT_dot_exp = -75.23724
        x_dot_exp = 342.4439
        y_dot_exp = -266.7707
        z_dot_exp = 248.1241

        @test isapprox(VT_dot_est*M2FT, VT_dot_exp, atol=0.05)
        @test isapprox(α_dot_est, α_dot_exp, atol=0.0005)
        @test isapprox(β_dot_est, β_dot_exp, atol=0.06)

        @test isapprox(ψ_dot, ψ_dot_exp, atol=1e-6)  # ψ_dot [rad/s]
        @test isapprox(θ_dot, θ_dot_exp, atol=1e-7)  # θ_dot [rad/s]
        @test isapprox(ϕ_dot, ϕ_dot_exp, atol=1e-6)  # ϕ_dot [rad/s]

        @test isapprox(p_dot, p_dot_exp, atol=0.06)   # p_dot [rad/s]
        @test isapprox(q_dot, q_dot_exp, atol=0.005)  # q_dot [rad/s]
        @test isapprox(r_dot, r_dot_exp, atol=0.005)  # r_dot [rad/s]

        @test isapprox(x_dot*M2FT, x_dot_exp, atol=1e-4)   # x_dot (ft/s)
        @test isapprox(y_dot*M2FT, y_dot_exp, atol=1e-4)  # y_dot (ft/s)
        @test isapprox(-z_dot*M2FT, z_dot_exp, atol=1e-4)   # z_dot (ft/s)
    end

    @testset "Morelli" begin
        # Note that conditions are less strict for Morelli because there are
        # differences in Morelli aerodynamic data wrt Stevens.

        # XDOT MORELLI
        # [RAD/S]
        α_dot_exp = -0.88123
        β_dot_exp = -0.45276
        ψ_dot_exp = 2.145926
        θ_dot_exp =  0.32508
        ϕ_dot_exp = 2.14593
        p_dot_exp = 12.91108
        q_dot_exp = 0.97006
        r_dot_exp = -0.55450
        # [FT/S]
        VT_dot_exp = -77.57521
        x_dot_exp = 342.44390
        y_dot_exp = -266.77068
        z_dot_exp = 248.12412
        # From morelli implementation
        u_dot_exp = 100.84
        v_dot_exp = -206.45
        w_dot_exp = -436.98

        thrust_exp = 1.5912e+04
        qbar_exp =  219.72  # psf
        mach_exp =  0.46436

        thrust = get_pfm(get_propulsion(ac)).forces[1]
        @test isapprox(thrust, thrust_exp*LBF2N, atol=0.5)
        mach = get_mach(aerostate)
        @test isapprox(mach, mach_exp, atol=0.000001)
        qbar = get_qinf(aerostate)
        @test isapprox(qbar, qbar_exp*PSF2PA, atol=0.5)

        @test isapprox(u_dot, u_dot_exp*FT2M, atol=0.01)
        @test isapprox(v_dot, v_dot_exp*FT2M, atol=5)
        @test isapprox(w_dot, w_dot_exp*FT2M, atol=0.0005)

        @test_broken isapprox(VT_dot_est*M2FT, VT_dot_exp, atol=0.05)
        @test isapprox(VT_dot_est*M2FT, VT_dot_exp, atol=5)
        @test isapprox(α_dot_est, α_dot_exp, atol=0.00005)
        @test isapprox(β_dot_est, β_dot_exp, atol=0.05)

        @test isapprox(ψ_dot, ψ_dot_exp, atol=1e-6)  # ψ_dot [rad/s]
        @test_broken isapprox(r[8], θ_dot_exp, atol=1e-7)  # θ_dot [rad/s]
        @test isapprox(θ_dot, θ_dot_exp, atol=1e-5)  # θ_dot [rad/s]
        @test_broken isapprox(r[9], ϕ_dot_exp, atol=1e-6)  # ϕ_dot [rad/s]
        @test isapprox(ϕ_dot, ϕ_dot_exp, atol=0.5)  # ϕ_dot [rad/s]

        @test_broken isapprox(p_dot, p_dot_exp, atol=0.06)   # p_dot [rad/s]
        @test isapprox(p_dot, p_dot_exp, atol=0.5)   # p_dot [rad/s]
        @test isapprox(q_dot, q_dot_exp, atol=0.005)  # q_dot [rad/s]
        @test isapprox(r_dot, r_dot_exp, atol=1.5)   # r_dot [rad/s]

        @test isapprox(x_dot*M2FT, x_dot_exp, atol=1e-4)   # x_dot (ft/s)
        @test isapprox(y_dot*M2FT, y_dot_exp, atol=1e-4)  # y_dot (ft/s)
        @test isapprox(-z_dot*M2FT, z_dot_exp, atol=1e-4)   # z_dot (ft/s)
    end
end
