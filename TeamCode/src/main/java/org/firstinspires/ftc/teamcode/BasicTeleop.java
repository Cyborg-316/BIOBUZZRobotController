package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BasicTeleop", group = "TeleOp")
public class BasicTeleop extends OpMode {
    WaxedLightlyWeatheredCutCopperStairsRobot robot = new WaxedLightlyWeatheredCutCopperStairsRobot();

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.setOffsets(0, 0, 0);
    }

    @Override
    public void loop() {
        robot.gamePadPower(gamepad1, gamepad2, telemetry);
    }
}
