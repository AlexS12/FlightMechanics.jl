using Test
using FlightMechanics
using FlightMechanics.Models
using FlightMechanics.Aircrafts

#  Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control
#  and simulation: dynamics, controls design, and autonomous systems. John Wiley
#  & Sons. (page 192)
pos = PositionEarth(0, 0, 0)
env = Environment(pos, atmos="ISAF16", wind="NoWind", grav="const")

ac = F16()
fcs = F16FCS()

h = 0.0 * M2FT
psi = 0.234077 # rad
gamma = 0.0
turn_rate = 0.3  # rad/s
tas_fts = 502.0  # ft/s

tas =  tas_fts * FT2M

# Initial conditions for trimmer
α0 = exp_α + 10*DEG2RAD
β0 = 0.
stick_lon0 = exp_de/25.0 + 0.5
stick_lat0 = exp_da/21.5 + 0.5
pedals0 = exp_dr/30.0 + 0.5
thtl0 = exp_thtl + 0.3

set_stick_lon(fcs, stick_lon0)
set_stick_lat(fcs, stick_lat0)
set_pedals(fcs, pedals0)
set_thtl(fcs, thtl0)

ac, aerostate, state, fcs = steady_state_trim(
    ac, fcs, env, tas, pos, psi, gamma, turn_rate, α0, β0, show_trace=false
    );


linear_system = Linearize(ac, state, aerostate, fcs, env);

A = get_A(linear_system)

println(A)

########################

function uvwdot_to_vabdot(u,v,w,u_dot,v_dot,w_dot,vt)

    vt_dot = (u*u_dot + v*v_dot + w*w_dot) / vt
    alpha_dot = (u*w_dot + w*u_dot) / (u^2 + w^2)
    beta_dot = (v_dot*vt - v*vt_dot) / (vt*sqrt(u^2 + v^2))

    return vt_dot, alpha_dot, beta_dot
end

##########################

x = get_sixdof_euler_fixed_mass_state(state)
x_ = [x[1], x[2], x[3], x[4], x[5], x[6], x[8], x[9]]     # [u, v, w, p, q, r, theta, phi]
u_ = [fcs.de.value, fcs.da.value, fcs.dr.value, fcs.thtl.value]
A1 = hcat(A[1:6, 1:6], A[1:6, 8:9])
A2 = hcat(A[8:9, 1:6], A[8:9, 8:9])
A_ = vcat(A1, A2)
x_dot = A_*x_ #+ B*u
u_dot = x_dot[1]
v_dot = x_dot[2]
w_dot = x_dot[3]
u, v, w = x_[1], x_[2], x_[3]
vt = sqrt(u^2 + v^2 + w^2)
println(vt)
vt_dot = (u*u_dot + v*v_dot + w*w_dot) / vt
alpha_dot = (u*w_dot - w*u_dot)/(u^2 + w^2)
beta_dot = (v_dot*vt - v*vt_dot)/(vt * sqrt(u^2 + w^2))
println(x_)
x_dot[1], x_dot[2], x_dot[3] = vt_dot*M2FT, alpha_dot, beta_dot
println(vt_dot)
println(x_dot)      # ft/s  rad   rad/sec  # with vt, alpha, beta

A_exp = [                                
    -0.09 -169.0 31.4 5e-4 -7.75 2e-3 -31.2 -7.73;
    -5e-4 -1.05 3e-4 -5e-4 0.903 -1e-4 0.0151 -0.0607;
    -1e-4 1.4e-4 -3.22e-7 2.48e-1 7e-6 -9.61e-1 -0.0032 1.3e-2;
    -3e-4 0.0578 -5.94e1 -3.19 -0.0469 1.64 .0 .0;
    1e-3 1.26 1e-3 0.0589 -1.66 -0.0175 .0 .0;
    5e-4 -0.617 8.88 -0.299 0.0123 -0.565 .0 .0;
    .0 .0 .0 .0 0.203 -0.979 .0 -0.3;
    .0 .0 .0 1.0 0.0508 0.0105 0.3 .0
]
x_exp = [502.0, 0.2485, 4.8e-4, -0.0155, 0.2934, 0.06071, 0.05185, 1.367]
x_dot_exp = A_exp*x_exp
println(x_dot_exp)
