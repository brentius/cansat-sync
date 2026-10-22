"""GPL-3.0-only. Hardware-independent CanSat flight and ground software core.

Pure logic with no device access, so the same modules run on the flight
computer, the ground station and a development laptop. SI units unless a name
says otherwise.

    atmosphere, flight_state, lora, downlink, telemetry, link   standard library only
    eskf, firstfix, coarse, windprofile, landing                also need NumPy

The NumPy modules are for the Raspberry Pi Zero 2 W: levelling and the first
fix from one image (coarse), the pose solver it calls (firstfix), navigation
(eskf), the wind filter and landing prediction. Nothing here runs on the
STM32F405, which handles timing, sensors and the radio. Two things here are
the contract its firmware has to meet: the packet layout in telemetry, and the
serial protocol between the two processors in link (specified in
wiki/stm32_pi_link_protocol.md, with test vectors in tests/link_vectors.json).
link is the Pi's end of that protocol and the reference for the STM32's end;
no STM32 firmware exists yet.
"""
