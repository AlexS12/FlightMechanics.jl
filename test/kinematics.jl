import FlightMechanics: rigid_body_velocity, rigid_body_acceleration

@static if VERSION < v"0.7.0-DEV.2005"
    using Base.Test
else
    using Test
end


# TEST rigid_body_velocity

# Test without angular velocity
vel_P = [10.0, 0.0, 0.0]
ω = [0.0, 0.0, 0.0]
r_PQ = [5.0, 5.0, 5.0]

exp_vel_Q = [10.0, 0.0, 0.0]
vel_Q = rigid_body_velocity(vel_P, ω, r_PQ)
@test isapprox(vel_Q, exp_vel_Q)

# Only angular velocity in Z axis
vel_P = [0.0, 0.0, 0.0]
ω = [0.0, 0.0, 1.0]
r_PQ = [5.0, 5.0, 5.0]

exp_vel_Q = [-sqrt(50) * sin(pi/4), sqrt(50) * cos(pi/4), 0.0]
vel_Q = rigid_body_velocity(vel_P, ω, r_PQ)
@test isapprox(vel_Q, exp_vel_Q)

# X linear velocity and Z angular velocity
vel_P = [10.0, 0.0, 0.0]
ω = [0.0, 0.0, 1.0]
r_PQ = [5.0, 5.0, 5.0]

exp_vel_Q = [-sqrt(50) * sin(pi/4) + 10.0, sqrt(50) * cos(pi/4), 0.0]
vel_Q = rigid_body_velocity(vel_P, ω, r_PQ)
@test isapprox(vel_Q, exp_vel_Q)
