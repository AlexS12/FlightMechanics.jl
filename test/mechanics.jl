using FlightMechanics

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


# TEST rigid_body_acceleration
# Only X linear acceleration
acc_P = [10.0, 0.0, 0.0]
ω = [0.0, 0.0, 0.0]
ω_dot = [0.0, 0.0, 0.0]
r_PQ = [5.0, 5.0, 5.0]

exp_accel_Q = [10.0, 0.0, 0.0]
accel_Q = rigid_body_acceleration(acc_P, ω, ω_dot, r_PQ)
@test isapprox(accel_Q, exp_accel_Q)

# Only Z angular acceleration
acc_P = [0.0, 0.0, 0.0]
ω = [0.0, 0.0, 0.0]
ω_dot = [0.0, 0.0, 1.0]
r_PQ = [5.0, 5.0, 5.0]

exp_accel_Q = [-sqrt(50) * sin(pi/4), sqrt(50) * cos(pi/4), 0.0]
accel_Q = rigid_body_acceleration(acc_P, ω, ω_dot, r_PQ)
@test isapprox(accel_Q, exp_accel_Q)

# Only Z angular velocity
acc_P = [0.0, 0.0, 0.0]
ω = [0.0, 0.0, 2.0]
ω_dot = [0.0, 0.0, 0.0]
r_PQ = [5.0, 5.0, 5.0]

exp_accel_Q = [-4.0 * sqrt(50) * sin(pi/4), -4.0 * sqrt(50) * cos(pi/4), 0.0]
accel_Q = rigid_body_acceleration(acc_P, ω, ω_dot, r_PQ)
@test isapprox(accel_Q, exp_accel_Q)

# Steiner theorem
# No point translation
point0 = [0, 0, 0]
point1 = point0
inertia = [1 0 0;
           0 2 0;
           0 0 3]
mass = 10
@test isapprox(steiner_inertia(point0, inertia, mass, point1), inertia)

# Point with no inertia
point0 = [0, 0, 0]
point1 = [10, 0, 0]
inertia = zeros(3, 3)
mass = 10

exp_inertia = [0       0     0;
               0    1000     0;
               0       0  1000]
@test isapprox(steiner_inertia(point0, inertia, mass, point1), exp_inertia)


# Point with no inertia
point0 = [0, 0, 0]
point1 = [10, 0, 0]
inertia = [1 0 0;
           0 2 0;
           0 0 3]
mass = 10

exp_inertia = [0       0     0;
               0    1000     0;
               0       0  1000] + inertia
@test isapprox(steiner_inertia(point0, inertia, mass, point1), exp_inertia)
