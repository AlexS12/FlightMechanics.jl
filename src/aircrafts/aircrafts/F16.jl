using FlightMechanics
using FlightMechanics.Models

##----------------------------------------------------------------------------------------------------
## imports

# Aerodynamics
import FlightMechanics.Models:
    calculate_aerodynamics

# Propulsion
import FlightMechanics.Models:
    get_pfm, get_cj, get_power, get_efficiency, get_tanks,
    get_engine_position, get_engine_orientation, get_engine_gyro_effects,
    calculate_engine

# Aircraft Model
import FlightMechanics.Models:
    get_name, get_wing_area, get_wing_span, get_chord, get_arp,
    get_empty_mass_props,
    set_stick_lon!, set_stick_lat!, set_pedals!, set_thtl!,
    set_controls_trimmer!, get_controls_trimmer, get_controls_ranges_trimmer


##----------------------------------------------------------------------------------------------------
## exports

# Aerodynamics
export F16Aerodynamics, calculate_aerodynamics

# Propulsion
export F16Engine,
    get_pfm, get_cj, get_power, get_efficiency, get_tanks,
    get_engine_position, get_engine_orientation,
    calculate_engine

# FCS
export F16FCS

# Aircraft Model
export F16,
    get_name,
    get_empty_mass_props,
    get_wing_area,
    get_wing_span,
    get_chord,
    get_arp


##----------------------------------------------------------------------------------------------------
## structs

# Aerodynamics
struct F16Aerodynamics<:Aerodynamics    # TODO: CDα, CLα, CYβ... could be included here
    wind_pfm::PointForcesMoments
    wind_coeff_pfm::PointForcesMoments
    body_pfm::PointForcesMoments
    body_coeff_pfm::PointForcesMoments
end

# Propulsion
struct F16Engine<:Engine
    pfm::PointForcesMoments
    cj::Number
    power::Number
    efficiency::Number
    tanks::Array{RigidSolid, 1}
    h::Array{T, 1} where T<:Number  # angular momentum [kg·m²/s]
end

# FCS
struct F16FCS<:FCS      # TODO: implement also rate limits and time constants
    # Cabin controls
    stick_longitudinal::RangeControl
    stick_lateral::RangeControl
    pedals::RangeControl
    thtl::RangeControl

    # Aerodynamic surfaces
    de::RangeControl
    da::RangeControl
    dr::RangeControl
    # Engine
    cpl::RangeControl
end

# Aircraft Model
struct F16<:Aircraft
    mass_props::RigidSolid
    pfm::PointForcesMoments
    aerodynamics::F16Aerodynamics
    propulsion::Propulsion
end


##----------------------------------------------------------------------------------------------------
## constants

const DE_MAX = 25.0  # deg
const DA_MAX = 20.0  # deg  #XXX: In Stevens' book says 21.5 deg (Appendix A Section A.4)
const DR_MAX = 30.0  # deg


##----------------------------------------------------------------------------------------------------
## Aerodynamics

α_data = [-10. -5. 0. 5. 10. 15. 20. 25. 30. 35. 40. 45.]  # deg
β_data = [0. 5. 10. 15. 20. 25. 30.]  # deg

de_data = [-24. -12.   0.  12.  24.]  # deg

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
CXq_data = [-0.267  -0.110   0.308   1.340   2.080   2.910   2.760   2.050   1.500   1.490   1.830   1.210]
CYr_data = [ 0.882   0.852   0.876   0.958   0.962   0.974   0.819   0.483   0.590   1.210  -0.493  -1.040]
# XXX: -2.27 seems an error in Stenvens. In Morelli it is -0.2270
# CYp_data = [-0.108  -0.108  -0.188   0.110   0.258   0.226   0.344   0.362   0.611   0.529   0.298  -2.270]
CYp_data = [-0.108  -0.108  -0.188   0.110   0.258   0.226   0.344   0.362   0.611   0.529   0.298  -0.227]
CZq_data = [-8.800 -25.800 -28.900 -31.400 -31.200 -30.700 -27.700 -28.200 -29.000 -29.800 -38.300 -35.300]
Clr_data = [-0.126  -0.026   0.063   0.113   0.208   0.230   0.319   0.437   0.680   0.100   0.447  -0.330]
Clp_data = [-0.360  -0.359  -0.443  -0.420  -0.383  -0.375  -0.329  -0.294  -0.230  -0.210  -0.120  -0.100]
Cmq_data = [-7.210  -0.540  -5.230  -5.260  -6.110  -6.640  -5.690  -6.000  -6.200  -6.400  -6.600  -6.000]
Cnr_data = [-0.380  -0.363  -0.378  -0.386  -0.370  -0.453  -0.550  -0.582  -0.595  -0.637  -1.020  -0.840]
Cnp_data = [ 0.061   0.052   0.052  -0.012  -0.013  -0.024   0.050   0.150   0.130   0.158   0.240   0.150]

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
# Stevens
Cl_data = [ 0.000  0.000  0.000  0.000  0.000  0.000  0.000  0.000  0.000  0.000  0.000  0.000;
           -0.001 -0.004 -0.008 -0.012 -0.016 -0.019 -0.020 -0.020 -0.015 -0.008 -0.013 -0.015;
           -0.003 -0.009 -0.017 -0.024 -0.030 -0.034 -0.040 -0.037 -0.016 -0.002 -0.010 -0.019;
           -0.001 -0.010 -0.020 -0.030 -0.039 -0.044 -0.050 -0.049 -0.023 -0.006 -0.014 -0.027;
            0.000 -0.010 -0.022 -0.034 -0.047 -0.046 -0.059 -0.061 -0.033 -0.036 -0.035 -0.035;
            0.007 -0.010 -0.023 -0.034 -0.049 -0.046 -0.068 -0.071 -0.060 -0.058 -0.062 -0.059;
            0.009 -0.011 -0.023 -0.037 -0.050 -0.047 -0.074 -0.079 -0.091 -0.076 -0.077 -0.076]
Cl_data = Cl_data'

# Morelli version
# Cl_data = [ 0.000  0.000  0.000  0.000  0.000  0.000  0.000  0.000  0.000  0.000  0.000  0.000;
#            -0.001 -0.004 -0.008 -0.012 -0.016 -0.022 -0.022 -0.021 -0.015 -0.008 -0.013 -0.015;
#            -0.003 -0.009 -0.017 -0.024 -0.030 -0.041 -0.045 -0.040 -0.016 -0.002 -0.010 -0.019;
#            -0.001 -0.010 -0.020 -0.030 -0.039 -0.054 -0.057 -0.054 -0.023 -0.006 -0.014 -0.027;
#             0.000 -0.010 -0.022 -0.034 -0.047 -0.060 -0.069 -0.067 -0.033 -0.036 -0.035 -0.035;
#             0.007 -0.010 -0.023 -0.034 -0.049 -0.063 -0.081 -0.079 -0.060 -0.058 -0.062 -0.059;
#             0.009 -0.011 -0.023 -0.037 -0.050 -0.068 -0.089 -0.088 -0.091 -0.076 -0.077 -0.076]
# Cl_data = Cl_data'

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
# Stevens
# rows: beta columns=alpha
cldr_data = [ 0.005  0.017  0.014  0.010 -0.005  0.009  0.019  0.005 -0.000 -0.005 -0.011  0.008;
              0.007  0.016  0.014  0.014  0.013  0.009  0.012  0.005  0.000  0.004  0.009  0.007;
              0.013  0.013  0.011  0.012  0.011  0.009  0.008  0.005 -0.002  0.005  0.003  0.005;  # Morelli diff
              0.018  0.015  0.015  0.014  0.014  0.014  0.014  0.015  0.013  0.011  0.006  0.001;
              0.015  0.014  0.013  0.013  0.012  0.011  0.011  0.010  0.008  0.008  0.007  0.003;
              0.021  0.011  0.010  0.011  0.010  0.009  0.008  0.010  0.006  0.005  0.000  0.001;
              0.023  0.010  0.011  0.011  0.011  0.010  0.008  0.010  0.006  0.014  0.020  0.000]
cldr_data /= DR_MAX
cldr_data = cldr_data'

# Morelli
# rows: beta columns=alpha
# cldr_data = [ 0.005  0.017  0.014  0.010 -0.005  0.009  0.019  0.005 -0.000 -0.005 -0.011  0.008;
#               0.007  0.016  0.014  0.014  0.013  0.009  0.012  0.005  0.000  0.004  0.009  0.007;
#               0.013  0.013  0.011  0.012  0.011  0.009  0.008  0.005  0.000  0.005  0.003  0.005;  # <<
#               0.018  0.015  0.015  0.014  0.014  0.014  0.014  0.015  0.013  0.011  0.006  0.001;
#               0.015  0.014  0.013  0.013  0.012  0.011  0.011  0.010  0.008  0.008  0.007  0.003;
#               0.021  0.011  0.010  0.011  0.010  0.009  0.008  0.010  0.006  0.005  0.000  0.001;
#               0.023  0.010  0.011  0.011  0.011  0.010  0.008  0.010  0.006  0.014  0.020  0.000]
# cldr_data /= DR_MAX
# cldr_data = cldr_data'

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
# Stevens
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

# Morelli
# rows: beta columns=alpha
# cnda_data = [ 0.001 -0.027 -0.017 -0.013 -0.012 -0.016  0.001  0.017  0.011  0.017  0.008  0.016;
#               0.002 -0.014 -0.016 -0.016 -0.014 -0.019 -0.021  0.002  0.012  0.015  0.016  0.011; # <<
#              -0.006 -0.008 -0.006 -0.006 -0.005 -0.008 -0.005  0.007  0.004  0.007  0.006  0.006;
#              -0.011 -0.011 -0.010 -0.009 -0.008 -0.006  0.000  0.004  0.007  0.010  0.004  0.010;
#              -0.015 -0.015 -0.014 -0.012 -0.011 -0.008 -0.002  0.002  0.006  0.012  0.011  0.011;
#              -0.024 -0.010 -0.004 -0.002 -0.001  0.003  0.014  0.006 -0.001  0.004  0.004  0.006;
#              -0.022  0.002 -0.003 -0.005 -0.003 -0.001 -0.009 -0.009 -0.001  0.003 -0.002  0.001]
# cnda_data /= DA_MAX
# cnda_data = cnda_data'

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

   α = get_alpha(aerostate)
   β = get_beta(aerostate)
   # Generate aerodynamics  object
   aerodynamics_from_body_coeff(adim_pfm_body, α, β, qinf, Sw, b, c, aero)
end


##----------------------------------------------------------------------------------------------------
## Propulsion

function F16Engine()
    F16Engine(
        PointForcesMoments(zeros(3), zeros(3), zeros(3)),
        0,
        0,
        0,
        # Fuel tanks can be obtained from JSBSim model
        [PointMass(0 * LB2KG, [0., 0., 0.] .* IN2M)],
        [160.0*SLUGFT2_2_KGM2, 0.0, 0.0]
        )

end

# TODO: Engine position and orientation
get_engine_position(prop::F16Engine) = [0.35 * 11.32 * FT2M, 0.0, 0.0]
get_engine_orientation(prop::F16Engine) = [0., 0., 0.] .* DEG2RAD
get_engine_gyro_effects(prop::F16Engine) = [160.0*SLUGFT2_2_KGM2, 0.0, 0.0]  # [kg·m²/s]


# ENGINE DATA
"""
    tgear(thtl)

Given the thottle setting (0 ≤ thtl ≤ 1) calculate the commanded power level
(0 ≤ cpow ≤ 100)
"""
function tgear(thtl)
    if thtl <= .77
     cpow = 64.94 * thtl
    else
     cpow = 217.38 * thtl - 117.38
    end
    return cpow
end

"""
    pdot(p3, p1)

Rate of change of power with time using a first order lag (%/s)

# Arguments
p3::Number  Actual power (%)
p1::Number  Power command (%)
"""
function pdot(p3, p1)
    if p1 >= 50.
        if p3 >= 50.
            t = 5.
            p2 = p1
        else
            p2 = 60.
            t = rtau(p2-p3)
        end
    else
        if p3 >= 50.
            t = 5.
            p2 = 40.
        else
            p2 = p1
            t = rtau(p2-p3)
        end
    end
    pd = t * (p2 - p3)
    return pd
end

"""
    rtau(dp)

Calculate the reciprocal time constant (1/s) for a first-order thrust lag given
the difference between the commanded power level (%) and the actual power
level (%).
"""
function rtau(dp)
    if dp <= 25.
        rt = 1.0  # Reciprocal time constant
    elseif dp >= 50.
        rt = 0.1
    else
        rt = 1.9 - 0.036 * dp
    end
    return rt
end


idle_data =
    [ 1060.  670.   880.   1140.  1500.  1860.
      635.   425.   690.   1010.  1330.  1700.
      60.    25.    345.   755.   1130.  1525.
     -1020. -710.  -300.   350.   910.   1360.
     -2700. -1900. -1300. -247.   600.   1100.
     -3600. -1400. -595.  -342.  -200.   700.]'

# Mil data
mil_data =
    [12680. 9150.  6200.  3950.  2450. 1400.
     12680. 9150.  6313.  4040.  2470. 1400.
     12610. 9312.  6610.  4290.  2600. 1560.
     12640. 9839.  7090.  4660.  2840. 1660.
     12390. 10176. 7750.  5320.  3250. 1930.
     11680. 9848.  8050.  6100.  3800. 2310.]'

# Max data
max_data =
    [20000. 15000. 10800. 7000.  4000. 2500.
     21420. 15700. 11225. 7323.  4435. 2600.
     22700. 16860. 12250. 8154.  5000. 2835.
     24240. 18910. 13760. 9285.  5700. 3215.
     26070. 21075. 15975. 11115. 6860. 3950.
     28886. 23319. 18300. 13484. 8642. 5057.]'


"""
    thrust(pow, alt, rmach)

Given the commanded power level (%), the altitude (ft) and the Mach number,
calculate the engine thrust (lbf)
"""
function calculate_thrust(pow, alt, rmach)

    h = 0.0001 * alt
    i = floor(Int, h)
    if i >= 5
        i=4
    end

    dh = h - float(i)

    rm = 5.0 * rmach
    m = floor(Int, rm);
    if m >= 5
        m = 4
    end

    dm = rm - float(m)
    cdh = 1.0 - float(dh)

    i=i+1;
    m=m+1;

    s = mil_data[i, m] * cdh + mil_data[i+1, m] * dh
    t = mil_data[i, m+1] *cdh + mil_data[i+1, m+1] * dh
    tmil = s + (t - s) * dm

    if pow < 50.
        s = idle_data[i, m] * cdh + idle_data[i+1, m] * dh
        t = idle_data[i,m+1] * cdh + idle_data[i+1, m+1] * dh
        tidl = s + (t - s) * dm
        thrst = tidl + (tmil - tidl) * pow * 0.02
    else
        s = max_data[i, m] * cdh + max_data[i+1, m] * dh
        t = max_data[i, m+1] * cdh + max_data[i+1, m+1] * dh
        tmax = s + (t-s) * dm
        thrst = tmil + (tmax - tmil) * (pow-50.) * 0.02
    end

    return thrst
    end


"""
The F-16 engine power response is modelled by a first-order lag (pdot function).
The rest of the model consists of thottle gearing (tgear) and lookup tables for
thrust as a function of operating power level, altitude and Mach.

Lookup tables have rows corresponding to Mach from 0 to 1 in increments of 0.2
and columns correspond to altitudes from 0 to 50000ft in increments of 10000ft.
There is a table for each power level: idle, military and maximum. Results can
be extrapolated beyond the boundaries but results may be unrealistic.

# References

Reimplemented from:

- [1] Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control
 and simulation: dynamics, controls design, and autonomous systems. John Wiley
 & Sons. (page 715)
"""
function calculate_engine(eng::F16Engine, fcs::FCS, aerostate::AeroState,
                          state::State; consume_fuel=false)

    pow = get_thrust(fcs)  # Commanded power: between 0 and 100
    Mach = get_mach(aerostate)
    # TODO: Should be altitude
    height = get_height(state) * M2FT

    thrust = calculate_thrust(pow, height, Mach)
    thrust *= LBF2N
    cj = 0.0
    Pm = thrust * get_tas(aerostate)
    ηp = NaN

    if consume_fuel==true
        error("fuel consumption not implemented")
    else
        tanks = get_tanks(eng)
    end

    # TODO: Torque calcualtion is missing
    pfm = PointForcesMoments(get_engine_position(eng),
                             [thrust, 0, 0],
                             [0, 0, 0]
                             )

    return typeof(eng)(pfm, cj, Pm, ηp, tanks, get_engine_gyro_effects(eng))
end


##----------------------------------------------------------------------------------------------------
## FCS

# TODO: Move to FCS in models
get_thrust(fcs::F16FCS) = get_value(fcs.cpl)

F16FCS() = F16FCS(# Cabin Inputs
                    RangeControl(0.0, [0, 1]),  # stick_longitudinal
                    RangeControl(0.0, [0, 1]),  # stick_lateral
                    RangeControl(0.0, [0, 1]),  # pedals
                    RangeControl(0.0, [0, 1]),  # thtl
                    # Controls
                    RangeControl(0.0, [-DE_MAX, DE_MAX] .* DEG2RAD),  # elevator
                    RangeControl(0.0, [-DA_MAX, DA_MAX] .* DEG2RAD),  # ailerons
                    RangeControl(0.0, [-DR_MAX, DR_MAX] .* DEG2RAD),  # rudder
                    # Commanded power level
                    RangeControl(0.0, [0.0, 100.]),                # CPL
                    )

function set_stick_lon!(fcs::F16FCS, value, allow_out_of_range=false, throw_error=false)
    set_value!(fcs.stick_longitudinal, value)
    min, max = get_value_range(fcs.de)
    range = max - min
    set_value!(fcs.de, min + range * value, allow_out_of_range, throw_error)
end

function set_stick_lat!(fcs::F16FCS, value, allow_out_of_range=false, throw_error=false)
    set_value!(fcs.stick_lateral, value)
    min, max = get_value_range(fcs.da)
    range = max - min
    set_value!(fcs.da, min + range * value, allow_out_of_range, throw_error)
end

function set_pedals!(fcs::F16FCS, value, allow_out_of_range=false, throw_error=false)
    set_value!(fcs.pedals, value)
    min, max = get_value_range(fcs.dr)
    range = max - min
    set_value!(fcs.dr, min + range * value, allow_out_of_range, throw_error)
end

function set_thtl!(fcs::F16FCS, value, allow_out_of_range=false, throw_error=false)
    set_value!(fcs.thtl, value)
    min, max = get_value_range(fcs.thtl)
    range = max - min
    set_value!(fcs.cpl, tgear(value), allow_out_of_range, throw_error)
end

function set_controls_trimmer!(fcs::F16FCS, slong, slat, ped, thtl,
    allow_out_of_range=true, throw_error=false)
    set_stick_lat!(fcs, slong, allow_out_of_range, throw_error)
    set_stick_lon!(fcs, slat, allow_out_of_range, throw_error)
    set_pedals!(fcs, ped, allow_out_of_range, throw_error)
    set_thtl!(fcs, thtl, allow_out_of_range, throw_error)
end

function get_controls_trimmer(fcs::F16FCS)
    [get_value(fcs.stick_longitudinal),
     get_value(fcs.stick_lateral),
     get_value(fcs.pedals),
     get_value(fcs.thtl)]
 end

function get_controls_ranges_trimmer(fcs::F16FCS)
    [get_value_range(fcs.stick_longitudinal),
     get_value_range(fcs.stick_lateral),
     get_value_range(fcs.pedals),
     get_value_range(fcs.thtl)]
end


##----------------------------------------------------------------------------------------------------
## Aircraft Model

function F16()
    pfm0 = PointForcesMoments(zeros(3), zeros(3), zeros(3))
    aero0 = F16Aerodynamics(pfm0, pfm0, pfm0, pfm0)
    engine = F16Engine()
    propulsion0 = Propulsion(
            PointForcesMoments(zeros(3), zeros(3), zeros(3)),
            0, 0, 0,
            [engine],
            get_gyro_effects(engine)
            )

    # mass properties cannot be retrieved until ac is created... so:
    ac = F16(RigidSolid(0, zeros(3), zeros(3, 3)),
              pfm0,
              aero0,
              propulsion0)
    mass = get_fuel_mass_props(get_propulsion(ac)) + get_empty_mass_props(ac)
            # + get_payload_mass_props(ac)
    F16(mass, pfm0, aero0, propulsion0)
end


# Name
function get_name(ac::F16)
    return "F16"
end

# GEOMETRIC PROPERTIES
get_wing_area(ac::F16) = 300.0 * FT2M^2
get_wing_span(ac::F16) = 30. * FT2M
get_chord(ac::F16) = 11.32 * FT2M

# Aerodynamic Reference Point
# Origin assumed leading edge of CMA
get_arp(ac::F16) = [-0.35 * get_chord(ac), 0.0, 0.0]
get_empty_cg(ac::F16) = [-0.35 * get_chord(ac), 0, 0]

# MASS PROPERTIES
function get_empty_mass_props(ac::F16)
    RigidSolid(
        20500 * LB2KG,                                # Empty mass
        get_empty_cg(ac),                             # Empty CG
        [9456.        0.    -982.;                     # Empty inertia
            0.    55814.       0.;
          -982.       0.   63100.]  .* SLUGFT2_2_KGM2
    )
end

# Crew and cargo
# get_pilot_mass_props(ac::F16) = PointMass(0 * LB2KG, [0., 0., 0.] .* IN2M)
# get_cargo_mass_props(ac::F16) = PointMass(0 * LB2KG, [0., 0., 0.] .* IN2M)

# get_payload_mass_props(ac::F16) = get_pilot_mass_props(ac)


##----------------------------------------------------------------------------------------------------
