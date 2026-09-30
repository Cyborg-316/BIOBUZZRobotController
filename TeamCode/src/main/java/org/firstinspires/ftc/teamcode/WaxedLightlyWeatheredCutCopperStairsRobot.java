package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class WaxedLightlyWeatheredCutCopperStairsRobot {
    public ElapsedTime driveModeDebounce = new ElapsedTime();
    public boolean gyroDrive = false;
    public double flDrivePower;
    public double frDrivePower;
    public double brDrivePower;
    public double blDrivePower;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    public Pose2D desiredPose = new Pose2D(DistanceUnit.MM, 0, 0, RADIANS, 0);
    public IMU imu;
//    public GoBildaPinpointDriver odo;

    public void init(final HardwareMap hardwareMap) {
        // Initialize hardware map - declares code version of physical motors using the driver/control hub
        frontLeft = hardwareMap.get(DcMotor.class, "flDrive"); //3
        frontRight = hardwareMap.get(DcMotor.class, "frDrive"); //2
        backLeft = hardwareMap.get(DcMotor.class, "blDrive"); //0
        backRight = hardwareMap.get(DcMotor.class, "brDrive"); //1

        // Set reverse motors
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //encoders
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setTargetPosition(0);
        backLeft.setTargetPosition(0);
        backRight.setTargetPosition(0);
        frontRight.setTargetPosition(0);

        //zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up the IMU (gyro/angle sensor)
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(imuParameters);

        this.imu.resetYaw();
        //Initializing odometry
//        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
    }

//UTILITY
    public void handleMotorPower() {
        frontLeft.setPower(flDrivePower);
        frontRight.setPower(frDrivePower);
        backLeft.setPower(blDrivePower);
        backRight.setPower(brDrivePower);
    }
    public void handleMotorPower(int wait) {
        handleMotorPower();
        sleep(wait);
    }
    public double yaw(AngleUnit units){
        return imu.getRobotYawPitchRollAngles().getYaw(units);
    }
//    public double odoHeading(AngleUnit units){
//        return odo.getHeading(units);
//    }

    public void updateTelemetry(Telemetry telemetry) {
//        telemetry.addData("FL Drive Power: ", flDrivePower);
//        telemetry.addData("FR Drive Power: ", frDrivePower);
//        telemetry.addData("BL Drive Power: ", blDrivePower);
//        telemetry.addData("BR Drive Power: ", brDrivePower);
//        telemetry.addData("Drive Mode: ", driveMode);
//        telemetry.addData("IMU-X", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, DEGREES).firstAngle);
//        telemetry.addData("IMU-Y", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, DEGREES).secondAngle);
//        telemetry.addData("IMU-Z", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, DEGREES).thirdAngle);
        telemetry.addData("IMU Angle: ", yaw(DEGREES));
//        telemetry.addData("Angle: ", odoHeading(DEGREES));
//        telemetry.addData("Camera: ", camera);
//        telemetry.addData("Position: ", odo.getPosition());

        telemetry.update();
    }

    public void brake() {
        flDrivePower = 0.0;
        blDrivePower = 0.0;
        frDrivePower = 0.0;
        brDrivePower = 0.0;

        frontLeft.setPower(0.0);
        backLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backRight.setPower(0.0);
    }

    public void sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException ignored) {
        }
    }

    private void setDriveMode(final DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }
    public void drive(final double pow) {
        //Sets all motors to pow
        frontLeft.setPower(pow);
        backLeft.setPower(pow);
        frontRight.setPower(pow);
        backRight.setPower(pow);
    }

    public void setOffsets(int initialX, int initialY, int initialDir){
//        odo.setOffsets(0, 0, DistanceUnit.MM);//was 70, 315 (smth is very wrong here - maybe the offsets are reversed? - -250, -175)
//        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
//        odo.resetPosAndIMU();
//        sleep(300);
//        odo.setPosition(new Pose2D(DistanceUnit.MM, initialX, initialY, DEGREES, initialDir));
    }

    public void resetOdo(Gamepad gp1) {
        //reset the odometry mid-match
        if (gp1.left_bumper && gp1.right_bumper) {
            this.imu.resetYaw();
            setOffsets(0, 0, 0);
        }
    }

    //TeleOp

    public void gamePadPower(Gamepad gp1, Gamepad gp2, Telemetry telemetry) {
//        odo.update();
//        camera.update();

        /****************** QUICK REFERENCE ******************/

        // left stick y-axis = gp1.left_stick_y
        // left stick x-axis = gp1.left_stick_x
        // right stick y-axis = gp1.right_stick_y
        // right stick x-axis = gp1.right_stick_x
    
        // left trigger = gp1.left_trigger
        // right trigger = gp1.right_trigger
        // left bumper = gp1.left_bumper
        // right bumper = gp1.right_bumper
    
        // a button = gp1.a
        // b button = gp1.b
        // x button = gp1.x
        // y button = gp1.y
    
        // dpad up = gp1.dpad_up
        // dpad down = gp1.dpad_down
        // dpad left = gp1.dpad_left
        // dpad right = gp1.dpad_right

        /****************** TASK ******************/

        // Code the following:
        // When B pressed, brake
        // When left trigger or right trigger pressed, speed up and slow down respectively
        // Have minimum power ex. 0.3, 0.4, etc.

        /****************** CODE STARTS HERE ******************/
        
        if (gp1.b) {
            brake();
        } else {
            double multiplier = 1.0;
            if (gp1.left_trigger > 0.1) {
                multiplier = 1.5;
            } else if (gp1.right_trigger > 0.1) {
                multiplier = 0.3;
            }

            final double drive = (-gp1.left_stick_y);
            final double turn = (gp1.right_stick_x);
            final double strafe = (gp1.left_stick_x);
    
            //math to determine power - uses mecanum wheel equation
            flDrivePower = (drive + strafe + turn);
            frDrivePower = (drive - strafe - turn);
            blDrivePower = (drive - strafe + turn);
            brDrivePower = (drive + strafe - turn);

            //spaget
            if (flDrivePower > 0) {
                flDrivePower = Math.max(0.3, flDrivePower);
            } else if (flDrivePower < 0){
                flDrivePower = Math.min(-0.3, flDrivePower);
            }

            if (frDrivePower > 0) {
                frDrivePower = Math.max(0.3, frDrivePower);
            } else if (frDrivePower < 0){
                frDrivePower = Math.min(-0.3, frDrivePower);
            }

            if (blDrivePower > 0) {
                blDrivePower = Math.max(0.3, blDrivePower);
            } else if (blDrivePower < 0){
                blDrivePower = Math.min(-0.3, blDrivePower);
            }

            if (brDrivePower > 0) {
                brDrivePower = Math.max(0.3, brDrivePower);
            } else if (brDrivePower < 0){
                brDrivePower = Math.min(-0.3, brDrivePower);
            }

            frontLeft.setPower(flDrivePower * multiplier);
            frontRight.setPower(frDrivePower * multiplier);
            backLeft.setPower(blDrivePower * multiplier);
            backRight.setPower(brDrivePower * multiplier);
        }
  

        /****************** CODE ENDS HERE ******************/
        
//        if (gp1.dpad_down){
//            desiredPose = odo.getPosition();
//        }
        toggleGyro(gp1);
//        resetOdo(gp1);
        updateTelemetry(telemetry);
    }

    public void toggleGyro(Gamepad gp1) {
        //Toggles gyro on or off

        //toggles GyroDrive boolean
        if (gp1.x) {
            if (driveModeDebounce.milliseconds() < 500) return;
            driveModeDebounce.reset();

            gyroDrive = !gyroDrive;
        }
    }

    public void driving(Gamepad gp1) {
        //no looking
        
    }

    public void gyroDrive(Gamepad gp1) {
        //no looking v2

    }

    public void staybot() {

//        double xmm = desiredPose.getX(DistanceUnit.MM);
//        double ymm = desiredPose.getY(DistanceUnit.MM);
//        double error = 15.0;
//        double errorButMore = 225.0 + error;
//
//        if (Math.abs(xmm - odo.getPosX(DistanceUnit.MM)) > error || Math.abs(ymm - odo.getPosY(DistanceUnit.MM)) > error) {
//            // Drive
//            double xDist = xmm - odo.getPosX(DistanceUnit.MM);
//            double yDist = xmm - odo.getPosY(DistanceUnit.MM);
//
//            double drivePwr = yDist / (double) Math.abs(ymm);
//            double strafePwr = xDist / (double) Math.abs(xmm);
//
//            if (Math.abs(yDist) > error) {
//                drivePwr = drivePwr + ((Math.signum(drivePwr) * 0.2));
//            } else {
//                drivePwr = 0.0;
//            }
//            if (Math.abs(xDist) > error) {
//                strafePwr = strafePwr + ((Math.signum(strafePwr) * 0.2));
//            } else {
//                strafePwr = 0.0;
//            }
//
//            if (Math.abs(xDist) > errorButMore) {
//                drivePwr = 1.2 * Math.signum(yDist);
//            }
//            if (Math.abs(yDist) > errorButMore) {
//                strafePwr = 1.2 * Math.signum(xDist);
//            }
//
//            double angle = odoHeading(RADIANS);
//
//            double cosine = Math.cos(-angle);
//            double sine = Math.sin(-angle);
//
//            double driveCos = cosine * drivePwr;
//            double driveSin = sine * drivePwr;
//            double strafeCos = cosine * strafePwr;
//            double strafeSin = sine * strafePwr;
//
//            drivePwr = (driveCos + strafeSin) / 1.25;
//            strafePwr = (strafeCos - driveSin);
//
//            frontLeft.setPower((drivePwr + strafePwr) / 1.5);
//            frontRight.setPower((drivePwr - strafePwr) / 1.5);
//            backLeft.setPower((drivePwr - strafePwr) / 1.5);
//            backRight.setPower((drivePwr + strafePwr) / 1.5);
//
//        } else {
//            brake();
//        }
    }

    //AUTO

    public void setX(int pos) {
//        double targetAngle = 0;
//
//        double currentAngle = yaw(DEGREES);
//        double errorAngle = targetAngle - currentAngle;
//
//        brake();
//
//        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        brake();
//
//        final int DELAY = 20;
//
//        int mm = 10 * pos;
//
//        int flBrPos = mm;
//        int frBlPos = -mm;
//
//        while (Math.abs(mm - odo.getPosX(DistanceUnit.MM)) > 5) {
//            double flDistance = flBrPos - odo.getPosX(DistanceUnit.MM);
//            double frDistance = frBlPos - odo.getPosX(DistanceUnit.MM);
//            double blDistance = frBlPos - odo.getPosX(DistanceUnit.MM);
//            double brDistance = flBrPos - odo.getPosX(DistanceUnit.MM);
//
//            flDrivePower = flDistance / (double) Math.abs(flBrPos);
//            blDrivePower = blDistance / (double) Math.abs(frBlPos);
//            frDrivePower = frDistance / (double) Math.abs(frBlPos);
//            brDrivePower = brDistance / (double) Math.abs(flBrPos);
//
//            if (Math.abs(flBrPos - backRight.getCurrentPosition()) > 100 || Math.abs(flBrPos - frontLeft.getCurrentPosition()) > 100) {
//                flDrivePower = Math.signum(flDrivePower);
//                frDrivePower = Math.signum(frDrivePower);
//                blDrivePower = Math.signum(blDrivePower);
//                brDrivePower = Math.signum(brDrivePower);
//            } else {
//                flDrivePower = (flDrivePower) + (Math.signum(flDrivePower) * 0.15);
//                frDrivePower = (frDrivePower) + (Math.signum(frDrivePower) * 0.15);
//                blDrivePower = (blDrivePower) + (Math.signum(blDrivePower) * 0.15);
//                brDrivePower = (brDrivePower) + (Math.signum(brDrivePower) * 0.15);
//            }
//
//            // Correct angle
//            currentAngle = yaw(DEGREES);
//            errorAngle = targetAngle - currentAngle;
//
//            double kp = 1;
//
//            double proportional = errorAngle * kp;
//
//            double turn = proportional / 180;
//
//            double flDrivePowerCorrection = -turn;
//            double frDrivePowerCorrection = turn;
//            double blDrivePowerCorrection = -turn;
//            double brDrivePowerCorrection = turn;
//
//            flDrivePowerCorrection = (flDrivePowerCorrection / 2) + (Math.signum(flDrivePowerCorrection) * 0.1);
//            frDrivePowerCorrection = (frDrivePowerCorrection / 2) + (Math.signum(frDrivePowerCorrection) * 0.1);
//            blDrivePowerCorrection = (blDrivePowerCorrection / 2) + (Math.signum(blDrivePowerCorrection) * 0.1);
//            brDrivePowerCorrection = (brDrivePowerCorrection / 2) + (Math.signum(brDrivePowerCorrection) * 0.1);
//
//            flDrivePower += flDrivePowerCorrection;
//            frDrivePower += frDrivePowerCorrection;
//            blDrivePower += blDrivePowerCorrection;
//            brDrivePower += brDrivePowerCorrection;
//
//            handleMotorPower();
//            odo.update();
//
//            sleep(DELAY);
//        }
//
//        brake();
//
//        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setY(int pos, double slowdown) {
//        if (pos == 0) return;
//
//        brake();
//
//        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        brake();
//
//        int mm = pos * 10;
//        final int DELAY = 20;
//        odo.update();
//
//        while (Math.abs(mm - odo.getPosY(DistanceUnit.MM)) > 5) {
//            // Drive
//            double distance = mm - odo.getPosY(DistanceUnit.MM);
//
//            double power = distance / (double) Math.abs(mm);
//
//            power = (power / slowdown) + (Math.signum(power) * 0.1);
//
//            flDrivePower = power;
//            frDrivePower = power;
//            blDrivePower = power;
//            brDrivePower = power;
//
//            handleMotorPower();
//            odo.update();
//
//
//            sleep(DELAY);
//        }
//
//        brake();
//
//        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setCoords(double xPos, double yPos, double desAngle){
//        brake();
//
//        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        brake();
//
//        double xmm = xPos;//*10 for cm input
//        double ymm = yPos;//*10 for cm input
//        final int delay = 10;
//        double error = 15.0;
//        double angError = 0.5;
//        double errorButMore = 225.0 + error;
//        double kp = 1;
//        odo.update();
//
//        while (Math.abs(xmm - odo.getPosX(DistanceUnit.MM)) > error || Math.abs(ymm - odo.getPosY(DistanceUnit.MM)) > error || Math.abs(desAngle - odoHeading(DEGREES)) > angError){
//            // Drive
//            double xDist = xmm - odo.getPosX(DistanceUnit.MM);
//            double yDist = ymm - odo.getPosY(DistanceUnit.MM);
//
//            double drivePwr = yDist / (double) Math.abs(ymm);
//            double strafePwr = xDist / (double) Math.abs(xmm);
//
//            if (Math.abs(yDist) > error) {
//                drivePwr = drivePwr + ((Math.signum(drivePwr) * 0.2));
//            } else {
//                drivePwr = 0.0;
//            }
//            if (Math.abs(xDist) > error) {
//                strafePwr = strafePwr + ((Math.signum(strafePwr) * 0.2));
//            } else {
//                strafePwr = 0.0;
//            }
//
//            if (Math.abs(xDist) > errorButMore){
//                drivePwr = 1.2 * Math.signum(yDist);
//            }
//            if (Math.abs(yDist) > errorButMore){
//                strafePwr = 1.2 * Math.signum(xDist);
//            }
//
//            double angle = odoHeading(RADIANS);
//
//            double cosine = Math.cos(-angle);
//            double sine = Math.sin(-angle);
//
//            double driveCos = cosine * drivePwr;
//            double driveSin = sine * drivePwr;
//            double strafeCos = cosine * strafePwr;
//            double strafeSin = sine * strafePwr;
//
//            drivePwr = (driveCos + strafeSin) / 1.25;
//            strafePwr = (strafeCos - driveSin);
//
//            flDrivePower = (drivePwr + strafePwr) / 1.5;
//            frDrivePower = (drivePwr - strafePwr) / 1.5;
//            blDrivePower = (drivePwr - strafePwr) / 1.5;
//            brDrivePower = (drivePwr + strafePwr) / 1.5;
//
//
//            double curAngError = desAngle - odoHeading(DEGREES);
//
//            if (Math.abs(curAngError) > angError) {
//                double proportional = curAngError * kp;
//                double turn = proportional / 180;
//                turn = turn + (Math.signum(curAngError) * 0.1);
//
//                flDrivePower -= turn;
//                frDrivePower += turn;
//                blDrivePower -= turn;
//                brDrivePower += turn;
//
//            }
//
//            handleMotorPower();
//            odo.update();
//
//            sleep(delay);
//        }
//
//        brake();
//
//        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void driveTo(int pos, double pct) {
        if (pos == 0) return;

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        brake();

        final int DELAY = 20;
        int error = 20;
        double addPwr = 0.1;

        while ((Math.abs(pos - frontLeft.getCurrentPosition()) + Math.abs(pos - frontRight.getCurrentPosition()) / 2) > error) {
            // Drive
            int flDistance = pos - frontLeft.getCurrentPosition();
            int frDistance = pos - frontRight.getCurrentPosition();
            int blDistance = pos - backLeft.getCurrentPosition();
            int brDistance = pos - backRight.getCurrentPosition();

            flDrivePower = (double) flDistance / (double) Math.abs(pos);
            blDrivePower = (double) blDistance / (double) Math.abs(pos);
            frDrivePower = (double) frDistance / (double) Math.abs(pos);
            brDrivePower = (double) brDistance / (double) Math.abs(pos);

            if (Math.abs(pos - frontLeft.getCurrentPosition()) > 150 && Math.abs(pos - frontRight.getCurrentPosition()) > 150) {
                flDrivePower = Math.signum(flDrivePower) * pct;
                frDrivePower = Math.signum(frDrivePower) * pct;
                blDrivePower = Math.signum(blDrivePower) * pct;
                brDrivePower = Math.signum(brDrivePower) * pct;
            } else {
                flDrivePower = (flDrivePower) + (Math.signum(flDrivePower) * addPwr);
                frDrivePower = (frDrivePower) + (Math.signum(frDrivePower) * addPwr);
                blDrivePower = (blDrivePower) + (Math.signum(blDrivePower) * addPwr);
                brDrivePower = (brDrivePower) + (Math.signum(brDrivePower) * addPwr);
            }

            // Slowdown
//            flDrivePower = flDrivePower;
//            frDrivePower = frDrivePower;
//            blDrivePower = blDrivePower;
//            brDrivePower = brDrivePower;

            handleMotorPower();

            sleep(DELAY);
        }

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void strafeTo(int pos) {
        double targetAngle = 0;

        double currentAngle = yaw(DEGREES);
        double errorAngle = targetAngle - currentAngle;

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        brake();

        final int DELAY = 20;
        int error = 20;
        double addPwr = 0.15;

        int flBrPos = pos;
        int frBlPos = -pos;

        while (Math.abs((Math.abs(flBrPos - frontLeft.getCurrentPosition()) + Math.abs(frBlPos - frontRight.getCurrentPosition())) / 2) > error) {
            int flDistance = flBrPos - frontLeft.getCurrentPosition();
            int frDistance = frBlPos - frontRight.getCurrentPosition();
            int blDistance = frBlPos - backLeft.getCurrentPosition();
            int brDistance = flBrPos - backRight.getCurrentPosition();

            flDrivePower = (double) flDistance / (double) Math.abs(flBrPos);
            blDrivePower = (double) blDistance / (double) Math.abs(frBlPos);
            frDrivePower = (double) frDistance / (double) Math.abs(frBlPos);
            brDrivePower = (double) brDistance / (double) Math.abs(flBrPos);

            if (Math.abs(flBrPos - backRight.getCurrentPosition()) > 100 || Math.abs(flBrPos - frontLeft.getCurrentPosition()) > 100) {
                flDrivePower = Math.signum(flDrivePower);
                frDrivePower = Math.signum(frDrivePower);
                blDrivePower = Math.signum(blDrivePower);
                brDrivePower = Math.signum(brDrivePower);
            } else {
                flDrivePower = (flDrivePower) + (Math.signum(flDrivePower) * addPwr);
                frDrivePower = (frDrivePower) + (Math.signum(frDrivePower) * addPwr);
                blDrivePower = (blDrivePower) + (Math.signum(blDrivePower) * addPwr);
                brDrivePower = (brDrivePower) + (Math.signum(brDrivePower) * addPwr);
            }

            // Correct angle
            currentAngle = yaw(DEGREES);
            errorAngle = targetAngle - currentAngle;

            double kp = 1;

            double proportional = errorAngle * kp;

            double turn = proportional / (180 * kp);

            double flDrivePowerCorrection = -turn;
            double frDrivePowerCorrection = turn;
            double blDrivePowerCorrection = -turn;
            double brDrivePowerCorrection = turn;

            flDrivePowerCorrection = (flDrivePowerCorrection / 2) + (Math.signum(flDrivePowerCorrection) * 0.1);
            frDrivePowerCorrection = (frDrivePowerCorrection / 2) + (Math.signum(frDrivePowerCorrection) * 0.1);
            blDrivePowerCorrection = (blDrivePowerCorrection / 2) + (Math.signum(blDrivePowerCorrection) * 0.1);
            brDrivePowerCorrection = (brDrivePowerCorrection / 2) + (Math.signum(brDrivePowerCorrection) * 0.1);

            flDrivePower += flDrivePowerCorrection;
            frDrivePower += frDrivePowerCorrection;
            blDrivePower += blDrivePowerCorrection;
            brDrivePower += brDrivePowerCorrection;

            // there was a slowdown here, I removed it cus not responsible, could be an issue later
            //TODO - see if this needs to be re-added

            handleMotorPower();

            sleep(DELAY);
        }

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnToFromHere(double target) {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentPosition = yaw(DEGREES);
        double error = target - currentPosition;

        double kp = 1;

        final int DELAY = 50;
        double allowedError = 1.0;
        double addPwr = 0.1;

        while (Math.abs(error) > allowedError) {
            currentPosition = yaw(DEGREES);
            error = target - currentPosition;

            double proportional = error * kp;

            double turn = proportional / (180 * kp);

            flDrivePower = -turn;
            frDrivePower = turn;
            blDrivePower = -turn;
            brDrivePower = turn;

            flDrivePower = (flDrivePower) + (Math.signum(flDrivePower) * addPwr);
            frDrivePower = (frDrivePower) + (Math.signum(frDrivePower) * addPwr);
            blDrivePower = (blDrivePower) + (Math.signum(blDrivePower) * addPwr);
            brDrivePower = (brDrivePower) + (Math.signum(brDrivePower) * addPwr);

            //removed slowdown stuff
            //TODO - check if this needs to be re-added

            handleMotorPower();

            sleep(DELAY);
        }

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void circleTurn(double centerX, double centerY)
    {   // variables
//        double startingX = odo.getPosX(DistanceUnit.MM);
//        double startingY = odo.getPosY(DistanceUnit.MM);
//        double radius = Math.sqrt(Math.pow((centerY-startingY),2)+Math.pow((centerX-startingX),2));
//        double strafe = 0.5;
//        double turn =  ((9.02694e-9)*Math.pow(radius,2))+(0.00327083*radius)+0.00219618; // got this from quadratic regression of measured points using donut function.
//        double drive = 0;
//
//        // changing Power
//        flDrivePower = (drive + strafe + turn);
//        frDrivePower = (drive - strafe - turn);
//        blDrivePower = (drive - strafe + turn);
//        brDrivePower = (drive + strafe - turn);
//        // actually puts power changes in effect
//        handleMotorPower(25);
    }


}
