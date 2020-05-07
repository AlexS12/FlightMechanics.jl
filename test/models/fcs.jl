using FlightMechanics.Models


spl_stream = StickPedalsLeverStream(
    ConstantInput(0.5),
    vcat(DoubletInput(0.5, 1, 2), ConstantInput(0.25)),
    vcat(RampInput(0.5, 1, 2), ConstantInput(0.1)),
    ConstantInput(0.9)
    )

controls = get_controls(spl_stream, 0.)

@test isapprox(controls.stick_lon, 0.5)
@test isapprox(controls.stick_lat, 0.25)
@test isapprox(controls.pedals, 0.1)
@test isapprox(controls.thrust_lever, 0.9)
