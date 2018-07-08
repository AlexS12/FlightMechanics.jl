import FlightMechanics: body2hor, hor2body, wind2hor, hor2wind, wind2body,
                        body2wind

@static if VERSION < v"0.7.0-DEV.2005"
    using Base.Test
else
    using Test
end

ones_ = [1.0, 1.0, 1.0]

# body2hor
@test ones_ ≈ body2hor(ones_..., 0., 0., 0.)
@test [2*0.70710678118654757, 1, 0] ≈ body2hor(ones_..., 0., 45*pi/180., 0.)
@test [1, 0, 2*0.70710678118654757] ≈ body2hor(ones_..., 0., 0., 45*pi/180)
@test [0, 2*0.70710678118654757, 1] ≈ body2hor(ones_..., 45*pi/180, 0., 0.)
# hor2body
@test ones_ ≈ hor2body(ones_..., 0., 0., 0.)
@test ones_ ≈ hor2body(2*0.70710678118654757, 1, 0, 0., 45*pi/180., 0.)
@test ones_ ≈ hor2body(1, 0, 2*0.70710678118654757, 0., 0., 45*pi/180)
@test ones_ ≈ hor2body(0, 2*0.70710678118654757, 1, 45*pi/180, 0., 0.)
#wind2hor
@test ones_ ≈ wind2hor(ones_..., 0., 0., 0.)
@test [2*0.70710678118654757, 1, 0] ≈ wind2hor(ones_..., 0., 45*pi/180., 0.)
@test [1, 0, 2*0.70710678118654757] ≈ wind2hor(ones_..., 0., 0., 45*pi/180)
@test [0, 2*0.70710678118654757, 1] ≈ wind2hor(ones_..., 45*pi/180, 0., 0.)
#hor2wind
@test ones_ ≈ hor2wind(ones_..., 0., 0., 0.)
@test ones_ ≈ hor2wind(2*0.70710678118654757, 1, 0, 0., 45*pi/180., 0.)
@test ones_ ≈ hor2wind(1, 0, 2*0.70710678118654757, 0., 0., 45*pi/180)
@test ones_ ≈ hor2wind(0, 2*0.70710678118654757, 1, 45*pi/180, 0., 0.)
#wind2body
@test ones_ ≈ wind2body(ones_..., 0., 0.)
@test [0, 1, 2*0.70710678118654757] ≈ wind2body(ones_..., 45*pi/180., 0.)
@test [0, 2*0.70710678118654757, 1] ≈ wind2body(ones_..., 0., 45*pi/180.)
#body2wind
@test ones_ ≈ body2wind(ones_..., 0., 0.)
@test ones_ ≈ body2wind(0, 1, 2*0.70710678118654757, 45*pi/180., 0.)
@test ones_ ≈ body2wind(0, 2*0.70710678118654757, 1, 0., 45*pi/180.)

