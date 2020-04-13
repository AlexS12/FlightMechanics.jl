using Test
using FlightMechanics
using FlightMechanics.Models
using FlightMechanics.Aircrafts


qinf = 10.
Sw = 5.
qSw = qinf*Sw
b = 100.
c = 10.
α, β = 0.0, 0.5

point = [1., 1., 1.]
fcoeff_body = [1., 2., 3.]
mcoeff_body = [-1., -2., -3.]

cfm_body = PointForcesMoments(point, fcoeff_body, mcoeff_body)
pfm_body = PointForcesMoments(point, qSw*fcoeff_body, qSw*[b, c, b].*mcoeff_body)

cfm_wind = PointForcesMoments(point,
                              body2wind(fcoeff_body..., α, β),
                              body2wind([b, c, b].*mcoeff_body..., α, β)./[b, c, b]
                              )
pfm_wind = PointForcesMoments(point,
                              body2wind(qSw*fcoeff_body..., α, β),
                              body2wind(qSw*[b, c, b].*mcoeff_body..., α, β)
                              )

aero_base = F16Aerodynamics(pfm_wind, cfm_wind, pfm_body, cfm_body)

aero1 = aerodynamics_from_body_total(pfm_body, α, β, qinf, Sw, b, c, aero_base)
aero2 = aerodynamics_from_body_coeff(cfm_body, α, β, qinf, Sw, b, c, aero_base)
aero3 = aerodynamics_from_wind_total(pfm_wind, α, β, qinf, Sw, b, c, aero_base)
aero4 = aerodynamics_from_wind_coeff(cfm_wind, α, β, qinf, Sw, b, c, aero_base)

for (ii, aero) in enumerate([aero1, aero2, aero3, aero4])
    @test isapprox(get_wind_pfm(aero), get_wind_pfm(aero_base), rtol=1e-5)
    @test isapprox(get_wind_adim_pfm(aero), get_wind_adim_pfm(aero_base), rtol=1e-5)
    @test isapprox(get_body_pfm(aero), get_body_pfm(aero_base), rtol=1e-5)
    @test isapprox(get_body_adim_pfm(aero), get_body_adim_pfm(aero_base), rtol=1e-5)
end
