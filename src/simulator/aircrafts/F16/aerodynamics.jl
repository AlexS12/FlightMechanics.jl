using FlightMechanics
using FlightMechanics.Simulator.Models
import FlightMechanics.Simulator.Models: calculate_aerodynamics

export F16Aerodynamics, calculate_aerodynamics


# TODO: CDα, CLα, CYβ... could be included here
struct F16Aerodynamics<:Aerodynamics
    wind_pfm::PointForcesMoments
    wind_coeff_pfm::PointForcesMoments
    body_pfm::PointForcesMoments
    body_coeff_pfm::PointForcesMoments
end


α_data = [-10. -5. 0. 5. 10. 15. 20. 25. 30. 35. 40. 45.]  # deg
β_data = [0. 5. 10. 15. 20. 25. 30.]  # deg

de_data = [-24. -12.   0.  12.  24.]  # deg

const DE_MAX = 25.0  # deg
const DA_MAX = 20.0  # deg  #XXX: In Stevens' book says 21.5 deg (Appendix A Section A.4)
const DR_MAX = 30.0  # deg

# UTILS for interpolations
function get_interp_idx(val, coeff, min_, max_, off_=3)
   s = val * coeff
   k = floor(Int, s)
   k = max(min_, k)
   k = min(k, max_)
   da = s - float(k)
   k += off_
   l = k + Int(sign(da))
   return k, l, da
end

function get_interp_idx2(val, coeff, min_, max_)
   s = abs(val) * coeff
   k = floor(Int, s)
   k = max(min_, k)
   k = min(k, max_)
   da = s - float(k)
   k += 1
   l = k + Int(sign(da))
   return k, l, da
end

function interp1d(val, coeff, min, max, data)
   k, l, da = get_interp_idx(val, coeff, min, max)
   res = data[k] + abs(da) * (data[l] - data[k])
   return res
end

function interp2d(val1, val2, coeff1, coeff2, min1, min2, max1, max2, data, off1=3, off2=3)
   k, l, da = get_interp_idx(val1, coeff1, min1, max1, off1)
   m, n, de = get_interp_idx(val2, coeff2, min2, max2, off2)

   t = data[k, m]
   u = data[k, n]
   v = t + abs(da) * (data[l, m] - t)
   w = u + abs(da) * (data[l, n] - u)
   res = v + (w - v) * abs(de)

   return res
end

function interp2d2(val1, val2, coeff1, coeff2, min1, min2, max1, max2, data)
   k, l, da = get_interp_idx(val1, coeff1, min1, max1)
   m, n, de = get_interp_idx2(val2, coeff2, min2, max2)

   t = data[k, m]
   u = data[k, n]
   v = t + abs(da) * (data[l, m] - t)
   w = u + abs(da) * (data[l, n] - u)
   res = v + (w - v) * abs(de)
   res *= sign(val2)

   return res
end


# DAMPING COEFFICIENTS
CXq_data = [-0.267 -0.110 0.308 1.340 2.080 2.910 2.760 2.050 1.500 1.490 1.830 1.210]
CYr_data = [0.882  0.852 0.876 0.958 0.962  0.974  0.819 0.483 0.590 1.210 -0.493 -1.040]
# XXX: -2.27 seems an error in Stenvens. In Morelli it is -0.2270
# CYp_data = [-0.108 -0.108 -0.188 0.110 0.258 0.226 0.344 0.362 0.611 0.529 0.298 -2.270]
CYp_data = [-0.108 -0.108 -0.188 0.110 0.258 0.226 0.344 0.362 0.611 0.529 0.298 -0.2270]
CZq_data = [-8.800 -25.800 -28.900 -31.400 -31.200 -30.700 -27.700 -28.200 -29.000 -29.800 -38.300 -35.300]
Clr_data = [-0.126 -0.026 0.063 0.113  0.208 0.230 0.319 0.437 0.680 0.100 0.447 -0.330]
Clp_data = [-0.360 -0.359 -0.443 -0.420 -0.383 -0.375 -0.329 -0.294 -0.230 -0.210 -0.120 -0.100]
Cmq_data = [-7.210 -0.540 -5.230 -5.260 -6.110 -6.640 -5.690 -6.000 -6.200 -6.400 -6.600 -6.000]
Cnr_data = [-0.380 -0.363 -0.378 -0.386 -0.370 -0.453 -0.550 -0.582 -0.595 -0.637 -1.020 -0.840]
Cnp_data = [0.061  0.052  0.052 -0.012 -0.013 -0.024  0.050  0.150 0.130  0.158  0.240  0.150]

damp_data = [CXq_data; CYr_data; CYp_data; CZq_data; Clr_data; Clp_data;
             Cmq_data; Cnr_data; Cnp_data]'
damp_data = damp_data'

# CX
# rows: de, columns: alpha
CX_data = [-0.099 -0.081 -0.081 -0.063 -0.025 0.044 0.097 0.113 0.145 0.167 0.174 0.166;
           -0.048 -0.038 -0.040 -0.021  0.016 0.083 0.127 0.137 0.162 0.177 0.179 0.167;
           -0.022 -0.020 -0.021 -0.004  0.032 0.094 0.128 0.130 0.154 0.161 0.155 0.138;
           -0.040 -0.038 -0.039 -0.025  0.006 0.062 0.087 0.085 0.100 0.110 0.104 0.091;
           -0.083 -0.073 -0.076 -0.072 -0.046 0.012 0.024 0.025 0.043 0.053 0.047 0.040]
CX_data = CX_data'

# CZ
# columns: alpha
CZ_data = [0.770 0.241 -0.100 -0.416 -0.731 -1.053 -1.366 -1.646 -1.917 -2.120 -2.248 -2.229]

# Cm
# rows: de columns: alpha
Cm_data = [0.205  0.168  0.186  0.196  0.213  0.251  0.245  0.238  0.252  0.231  0.198  0.192;
           0.081  0.077  0.107  0.110  0.110  0.141  0.127  0.119  0.133  0.108  0.081  0.093;
          -0.046 -0.020 -0.009 -0.005 -0.006  0.010  0.006 -0.001  0.014  0.000 -0.013  0.032;
          -0.174 -0.145 -0.121 -0.127 -0.129 -0.102 -0.097 -0.113 -0.087 -0.084 -0.069 -0.006;
          -0.259 -0.202 -0.184 -0.193 -0.199 -0.150 -0.160 -0.167 -0.104 -0.076 -0.041 -0.005]
Cm_data = Cm_data'

# Cl
# rows: beta columns: alpha
# XXX: found differences in Morelli Cl_data for columns 6-9
Cl_data = [ 0.000  0.000  0.000  0.000  0.000  0.000  0.000  0.000  0.000  0.000  0.000  0.000;
           -0.001 -0.004 -0.008 -0.012 -0.016 -0.019 -0.020 -0.020 -0.015 -0.008 -0.013 -0.015;
           -0.003 -0.009 -0.017 -0.024 -0.030 -0.034 -0.040 -0.037 -0.016 -0.002 -0.010 -0.019;
           -0.001 -0.010 -0.020 -0.030 -0.039 -0.044 -0.050 -0.049 -0.023 -0.006 -0.014 -0.027;
            0.000 -0.010 -0.022 -0.034 -0.047 -0.046 -0.059 -0.061 -0.033 -0.036 -0.035 -0.035;
            0.007 -0.010 -0.023 -0.034 -0.049 -0.046 -0.068 -0.071 -0.060 -0.058 -0.062 -0.059;
            0.009 -0.011 -0.023 -0.037 -0.050 -0.047 -0.074 -0.079 -0.091 -0.076 -0.077 -0.076]
Cl_data = Cl_data'
# rows: beta columns=alpha
clda_data = [-0.041 -0.052 -0.053 -0.056 -0.050 -0.056 -0.082 -0.059 -0.042 -0.038 -0.027 -0.017;
             -0.041 -0.053 -0.053 -0.053 -0.050 -0.051 -0.066 -0.043 -0.038 -0.027 -0.023 -0.016;
             -0.042 -0.053 -0.052 -0.051 -0.049 -0.049 -0.043 -0.035 -0.026 -0.016 -0.018 -0.014;
             -0.040 -0.052 -0.051 -0.052 -0.048 -0.048 -0.042 -0.037 -0.031 -0.026 -0.017 -0.012;
             -0.043 -0.049 -0.048 -0.049 -0.043 -0.042 -0.042 -0.036 -0.025 -0.021 -0.016 -0.011;
             -0.044 -0.048 -0.048 -0.047 -0.042 -0.041 -0.020 -0.028 -0.013 -0.014 -0.011 -0.010;
             -0.043 -0.049 -0.047 -0.045 -0.042 -0.037 -0.003 -0.013 -0.010 -0.003 -0.007 -0.008]
clda_data /= DA_MAX
clda_data = clda_data'
# rows: beta columns=alpha
cldr_data = [ 0.005  0.017  0.014  0.010 -0.005  0.009  0.019  0.005 -0.000 -0.005 -0.011  0.008;
              0.007  0.016  0.014  0.014  0.013  0.009  0.012  0.005  0.000  0.004  0.009  0.007;
              0.013  0.013  0.011  0.012  0.011  0.009  0.008  0.005 -0.002  0.005  0.003  0.005;
              0.018  0.015  0.015  0.014  0.014  0.014  0.014  0.015  0.013  0.011  0.006  0.001;
              0.015  0.014  0.013  0.013  0.012  0.011  0.011  0.010  0.008  0.008  0.007  0.003;
              0.021  0.011  0.010  0.011  0.010  0.009  0.008  0.010  0.006  0.005  0.000  0.001;
              0.023  0.010  0.011  0.011  0.011  0.010  0.008  0.010  0.006  0.014  0.020  0.000]
cldr_data /= DR_MAX
cldr_data = cldr_data'

# Cn
# rows: beta columns: alpha
Cn_data = [0.000  0.000  0.000  0.000  0.000  0.000  0.000  0.000 0.000  0.000  0.000  0.000;
           0.018  0.019  0.018  0.019  0.019  0.018  0.013  0.007 0.004 -0.014 -0.017 -0.033;
           0.038  0.042  0.042  0.042  0.043  0.039  0.030  0.017 0.004 -0.035 -0.047 -0.057;
           0.056  0.057  0.059  0.058  0.058  0.053  0.032  0.012 0.002 -0.046 -0.071 -0.073;
           0.064  0.077  0.076  0.074  0.073  0.057  0.029  0.007 0.012 -0.034 -0.065 -0.041;
           0.074  0.086  0.093  0.089  0.080  0.062  0.049  0.022 0.028 -0.012 -0.002 -0.013;
           0.079  0.090  0.106  0.106  0.096  0.080  0.068  0.030 0.064  0.015  0.011 -0.001]
Cn_data = Cn_data'
# rows: beta columns=alpha
cnda_data = [ 0.001 -0.027 -0.017 -0.013 -0.012 -0.016  0.001  0.017  0.011  0.017  0.008  0.016;
              0.002 -0.014 -0.016 -0.016 -0.014 -0.019 -0.021  0.002  0.012  0.015  0.015  0.011;
             -0.006 -0.008 -0.006 -0.006 -0.005 -0.008 -0.005  0.007  0.004  0.007  0.006  0.006;
             -0.011 -0.011 -0.010 -0.009 -0.008 -0.006  0.000  0.004  0.007  0.010  0.004  0.010;
             -0.015 -0.015 -0.014 -0.012 -0.011 -0.008 -0.002  0.002  0.006  0.012  0.011  0.011;
             -0.024 -0.010 -0.004 -0.002 -0.001  0.003  0.014  0.006 -0.001  0.004  0.004  0.006;
             -0.022  0.002 -0.003 -0.005 -0.003 -0.001 -0.009 -0.009 -0.001  0.003 -0.002  0.001]
cnda_data /= DA_MAX
cnda_data = cnda_data'
# rows: beta columns=alpha
cndr_data = [-0.018 -0.052 -0.052 -0.052 -0.054 -0.049 -0.059 -0.051 -0.030 -0.037 -0.026 -0.013;
             -0.028 -0.051 -0.043 -0.046 -0.045 -0.049 -0.057 -0.052 -0.030 -0.033 -0.030 -0.008;
             -0.037 -0.041 -0.038 -0.040 -0.040 -0.038 -0.037 -0.030 -0.027 -0.024 -0.019 -0.013;
             -0.048 -0.045 -0.045 -0.045 -0.044 -0.045 -0.047 -0.048 -0.049 -0.045 -0.033 -0.016;
             -0.043 -0.044 -0.041 -0.041 -0.040 -0.038 -0.034 -0.035 -0.035 -0.029 -0.022 -0.009;
             -0.052 -0.034 -0.036 -0.036 -0.035 -0.028 -0.024 -0.023 -0.020 -0.016 -0.010 -0.014;
             -0.062 -0.034 -0.027 -0.028 -0.027 -0.027 -0.023 -0.023 -0.019 -0.009 -0.025 -0.010]
cndr_data /= DR_MAX
cndr_data = cndr_data'


"""
   dampings(aero::F16Aerodynamics, α)

Calculates CX_q, CY_r, CY_p, CZ_q, Cl_r, Cl_p, Cm_q, Cn_r, Cn_p for the given
angle of attack, α (deg).
"""
function dampings(aero::F16Aerodynamics, α)
   # k, l, da = get_interp_idx(α, 0.2, -1, 8)
   # dampings = damp_data[k, :] + abs(da) * (damp_data[l, :] - damp_data[k, :])
   # TODO: check if vectorized approach is faster than loop (it shouldn't)
   dampings = Array{Float64}(undef, 9)
   for ii in 1:9
      dampings[ii] = interp1d(α, 0.2, -1, 8, damp_data[ii, :])
   end
   return dampings
end


Cl_da(aero::F16Aerodynamics, α, β) = interp2d(α, β, 0.2, 0.1, -1, -2, 8, 2, clda_data, 3, 4)
Cl_dr(aero::F16Aerodynamics, α, β) = interp2d(α, β, 0.2, 0.1, -1, -2, 8, 2, cldr_data, 3, 4)
Cn_da(aero::F16Aerodynamics, α, β) = interp2d(α, β, 0.2, 0.1, -1, -2, 8, 2, cnda_data, 3, 4)
Cn_dr(aero::F16Aerodynamics, α, β) = interp2d(α, β, 0.2, 0.1, -1, -2, 8, 2, cndr_data, 3, 4)


CXαde(aero::F16Aerodynamics, α, de) = interp2d(α, de, .2, 1.0/12, -1, -1, 8, 1, CX_data)

CYdr(aero::F16Aerodynamics, dr) = 0.086 / DR_MAX * dr
CYda(aero::F16Aerodynamics, da) = 0.021 / DA_MAX * da
CYβ(aero::F16Aerodynamics, β) = -0.02 * β

CZα(aero::F16Aerodynamics, α) = interp1d(α, 0.2, -1, 8, CZ_data)
CZαβ(aero::F16Aerodynamics, α, β) = CZα(aero, α) * (1. - (β*DEG2RAD)^2)
CZde(aero::F16Aerodynamics, de) = -0.19/DE_MAX * de

Clαβ(aero::F16Aerodynamics, α, β) = interp2d2(α, β, .2, .2, -1, 1, 8, 5, Cl_data)
Cldr(aero::F16Aerodynamics, α, β, dr) = Cl_dr(aero, α, β) * dr
Clda(aero::F16Aerodynamics, α, β, da) = Cl_da(aero, α, β) * da

Cmαde(aero::F16Aerodynamics, α, de) = interp2d(α, de, .2, 1.0/12, -1, -1, 8, 1, Cm_data)

Cnαβ(aero::F16Aerodynamics, α, β) = interp2d2(α, β, .2, .2, -1, 1, 8, 5, Cn_data)
Cndr(aero::F16Aerodynamics, α, β, dr) = Cn_dr(aero, α, β) * dr
Cnda(aero::F16Aerodynamics, α, β, da) = Cn_da(aero, α, β) * da



function calculate_aerodynamics(ac::Aircraft, aero::F16Aerodynamics, fcs::FCS,
   aerostate::AeroState, state::State)

   ARP = get_arp(ac)

   qinf = get_qinf(aerostate)  # Pa
   Sw = get_wing_area(ac)  # m²
   b = get_wing_span(ac)  # m
   c = get_chord(ac)  # m

   de = get_value(fcs.de) * RAD2DEG
   da = get_value(fcs.da) * RAD2DEG
   dr = get_value(fcs.dr) * RAD2DEG

   α = get_alpha(aerostate) * RAD2DEG
   β = get_beta(aerostate) * RAD2DEG
   tas = get_tas(aerostate) * M2FT
   # αdot = get_alpha_dot(aerostate)  # rad/s
   p, q, r = get_body_ang_velocity(state)  # rad/s

   c2v = c * M2FT / (2*tas)
   qc2v = q * c2v
   b2v = b * M2FT / (2*tas)

   CX_q, CY_r, CY_p, CZ_q, Cl_r, Cl_p, Cm_q, Cn_r, Cn_p = dampings(aero, α)

   CX = CXαde(aero, α, de) + CX_q * qc2v
   CY = CYβ(aero, β) + CYda(aero, da) + CYdr(aero, dr) + (CY_r*r + CY_p*p) * b2v
   CZ = CZαβ(aero, α, β) + CZde(aero, de) + CZ_q * qc2v

   Cl = Clαβ(aero, α, β) +  Clda(aero, α, β, da) + Cldr(aero, α, β, dr) +
       (Cl_r*r + Cl_p*p) * b2v
   Cm = Cmαde(aero, α, de) + Cm_q*qc2v
      # + CZ * (xcgr - xcg) is taken into account in pfm
   Cn = Cnαβ(aero, α, β) + Cnda(aero, α, β, da) + Cndr(aero, α, β, dr) +
       (Cn_r*r + Cn_p*p) * b2v
      # - CY * (xcgr - xcg) * c/b is taken into account in pfm

   # Pack results
   adim_pfm_body = PointForcesMoments(ARP, [CX, CY, CZ], [Cl, Cm, Cn])
   pfm_body = PointForcesMoments(ARP, qinf*Sw*[CX, CY, CZ],
                                      qinf*Sw*[b*Cl, c*Cm, b*Cn])
   α = get_alpha(aerostate)
   β = get_beta(aerostate)

   pfm_wind = rotate(pfm_body, -β, -α, 0)
   adim_pfm_wind = rotate(adim_pfm_body, -β, -α, 0)

   return F16Aerodynamics(pfm_wind, adim_pfm_wind, pfm_body, adim_pfm_body)
end
