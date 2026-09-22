package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BasicTeleOp", group = "TeleOp")
public class BasicTeleOp extends OpMode {
    PotatoRobot robot = new PotatoRobot();

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
