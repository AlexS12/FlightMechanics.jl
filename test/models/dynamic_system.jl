using FlightMechanics.Models


@testset "Dynamic systems conversions" begin
    pos = EarthPosition(1000.0, 500.0, -1000.0)
    att = Attitude(π/4, π/32, π/8)
    vel = [100.0, 0.0, 0.0]
    ang_vel = [π/90, π/18, π/9]
    accel = [1.5, 0.3, 0.5]
    ang_accel = [π/45, π/60, π/30]
    state = State(pos, att, vel, ang_vel, accel, ang_accel)

    # SixDOFEulerFixedMass
    x = convert(SixDOFEulerFixedMass, state)
    state_ = convert(state, x)
    @test isapprox(state, state_)

    # SixDOFQuaternionFixedMass
    x = convert(SixDOFQuaternionFixedMass, state)
    state_ = convert(state, x)
    @test isapprox(state, state_)

    # SixDOFAeroEulerFixedMass
    x = convert(SixDOFAeroEulerFixedMass, state)
    state_ = convert(state, x)
    @test isapprox(state, state_)

    # SixDOFECEFQuaternionFixedMass
    pos_ecef = convert(ECEFPosition, pos)
    state = State(pos_ecef, att, vel, ang_vel, accel, ang_accel)
    x = convert(SixDOFECEFQuaternionFixedMass, state)
    state_ = convert(state, x)
    @test isapprox(state, state_)
end


@testset "Dynamic systems evaluation" begin
    ac = F16()

    att = Attitude(250*DEG2RAD, 20*DEG2RAD, -30*DEG2RAD)
    pos = EarthPosition(0.0, 0.0, 0.0)
    u, v, w = wind2body(500*FT2M, 0, 0, 0.5, -0.2)
    
    state = State(
        pos,
        att,
        [u, v, w],
        [0.7, -0.8, 0.9],
        [0., 0., 0.],
        [0., 0., 0.]
    )

    # Modify directly the deflexions of the surfaces on the FCS
    fcs = get_fcs(ac)
    set_value!(fcs.de, 20.0*DEG2RAD)
    set_value!(fcs.da, -15.0*DEG2RAD)
    set_value!(fcs.dr, -20*DEG2RAD)
    set_value!(fcs.cpl, 90.0)

    env = Environment(pos, atmos="ISAF16", wind="NoWind", grav="const")
    aerostate = AeroState(state, env)
    grav = get_gravity(env)

    ac = calculate_aircraft(ac, aerostate, state, grav; consume_fuel=false)
    pfm = ac.pfm
    mass_props = get_mass_props(ac)
    mass = mass_props.mass
    inertia = mass_props.inertia
    h = [160.0*SLUGFT2_2_KGM2, 0.0, 0.0]

    ds_1 = convert(SixDOFEulerFixedMass, state)
    x_1 = get_x(ds_1)
    f = get_state_equation(ds_1)
    x_dot_1 = f(x_1, mass, inertia, pfm.forces, pfm.moments, h)
    ds = SixDOFEulerFixedMass(x_1, x_dot_1)
    st1 = convert(state, ds)

    ds_2 = convert(SixDOFAeroEulerFixedMass, state)
    x_2 = get_x(ds_2)
    f = get_state_equation(ds_2)
    x_dot_2 = f(x_2, mass, inertia, pfm.forces, pfm.moments, h)
    ds = SixDOFAeroEulerFixedMass(x_2, x_dot_2)
    st2 = convert(state, ds)

    ds_3 = convert(SixDOFQuaternionFixedMass, state)
    x_3 = get_x(ds_3)
    f = get_state_equation(ds_3)
    x_dot_3 = f(x_3, mass, inertia, pfm.forces, pfm.moments, h)
    ds = SixDOFQuaternionFixedMass(x_3, x_dot_3)
    st3 = convert(state, ds)

    ds_4 = convert(SixDOFECEFQuaternionFixedMass, state)
    x_4 = get_x(ds_4)
    f = get_state_equation(ds_4)
    x_dot_4 = f(x_4, mass, inertia, pfm.forces, pfm.moments, h)
    ds = SixDOFECEFQuaternionFixedMass(x_4, x_dot_4)
    st4 = convert(state, ds)
    # Note: Transform position to Earth position to be comparable with other states
    st4 = State(
        convert(st1.position, st4.position),
        st4.attitude,
        st4.velocity,
        st4.angular_velocity,
        st4.acceleration,
        st4.angular_acceleration
    )

    @testset "$(pair[1])" for pair in (
            ("SixDOFEulerFixedMass-SixDOFAeroEulerFixedMass", st1, st2),
            ("SixDOFEulerFixedMass-SixDOFQuaternionFixedMass", st1, st3),
            ("SixDOFAeroEulerFixedMass-SixDOFQuaternionFixedMass", st2, st3),
        )
        x, y = pair[2:3]
        @test isapprox(x.position, y.position)
        @test isapprox(x.attitude, y.attitude)
        @test isapprox(x.velocity, y.velocity)
        @test isapprox(x.angular_velocity, y.angular_velocity)
        @test isapprox(x.acceleration, y.acceleration)
        @test isapprox(x.angular_acceleration, y.angular_acceleration)
    end


    @testset "$(pair[1])" for pair in (
            ("SixDOFEulerFixedMass-SixDOFECEFQuaternionFixedMass", st1, st4),
            ("SixDOFAeroEulerFixedMass-SixDOFECEFQuaternionFixedMass", st2, st4),
            ("SixDOFQuaternionFixedMass-SixDOFECEFQuaternionFixedMass", st3, st4),
        )
        x, y = pair[2:3]
        @test isapprox(x.position, y.position)
        @test isapprox(x.attitude, y.attitude)
        @test isapprox(x.velocity, y.velocity)
        @test isapprox(x.angular_velocity, y.angular_velocity)
        # TODO: If ROT_VELOCITY in constants is set to zero this tests passes with atol=0.0
        # Otherwise the systems are not comparable because of the effect of rotating Earth.
        @test isapprox(x.acceleration, y.acceleration, atol=0.01)
        @test_broken isapprox(x.acceleration, y.acceleration)
        
        @test isapprox(x.angular_acceleration, y.angular_acceleration)
    end

end
