from g1_controller import G1ArmController, ControllerConfig, Gains

if __name__ == "__main__":
    cfg = ControllerConfig(
        network_interface="eth0",
        dt=0.02,
        gains=Gains(kp=60.0, kd=1.5),
        verbose=True,
    )

    arm = G1ArmController()
    arm.start(wait=True)

    arm.set_axes({22:90, 15:90, 25:60, 18:60, 28:-30, 21:-30, 27:0, 20:0, 16:-20, 23:-20, 26:-50, 19:-50}, 3.0)

    arm.set_all_axes_to_zero(duration=2.0)
    arm.stop()
