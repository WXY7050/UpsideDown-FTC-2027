package org.firstinspires.ftc.teamcode.upsidedown.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.upsidedown.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.upsidedown.subsystems.DriveSubsystem;

@TeleOp(name = "UpsideDown Auto Aim Model V2.1", group = "Test")
public class UpsideDownAutoAimModelV21 extends OpMode {

    private Follower follower;
    private DriveSubsystem drive;
    private RobotHardware robot;
    private VoltageSensor voltageSensor;

    private static final double START_X = 12.441 / 2.0;
    private static final double START_Y = 12.835 / 2.0;
    private static final double START_HEADING_DEG = 90.0;

    private static final double HIVE_X = 57.0;
    private static final double HIVE_Y = 55.0;
    private static final double HIVE_OPEN_HEADING_DEG = 270.0;

    public static double MODEL_MIN_DISTANCE = 36.0;
    public static double MODEL_MAX_DISTANCE = 55.0;
    public static double MODEL_MIN_SECTOR = -50.0;
    public static double MODEL_MAX_SECTOR = 40.0;

    public static double HEADING_TOLERANCE_DEG = 1.5;
    public static double AIM_KP = 0.018;
    public static double AIM_MAX_POWER = 0.55;
    public static double AIM_MIN_POWER = 0.07;

    private boolean fieldCentric = true;
    private boolean lastFieldCentricButton = false;

    public static double TARGET_RPM = 3750;
    public static double RPM_TOLERANCE = 35;
    public static final double FEED_POWER = 0.45;

    public static double kF = 0.00017020;
    public static double kP = 0.000446;
    public static double kI = 0.0;
    public static double kD = 0.000001;
    public static double MAX_INTEGRAL_OUTPUT = 0.20;

    public static final double NOMINAL_VOLTAGE = 12.9;
    public static double LED_DEEP_BLUE = 0.615;

    private double integral = 0.0;
    private double lastError = 0.0;
    private long lastTime = 0;
    private boolean feedUnlocked = false;

    @Override
    public void init() {
        robot = new RobotHardware();
        robot.init(hardwareMap);

        follower = Constants.create(hardwareMap);
        drive = new DriveSubsystem(hardwareMap);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        robot.block.setPosition(RobotHardware.BLOCK_CLOSE);
        robot.ledL.setPosition(0.48);
        robot.ledR.setPosition(0.48);

        telemetry.addData("状态", "Auto Aim Model V2.1 初始化完成");
        telemetry.addData("", "Start 后请确认机器人处于测试起始位");
        telemetry.update();
    }

    @Override
    public void start() {
        resetPID();
        feedUnlocked = false;

        Pose startPose = new Pose(
                START_X,
                START_Y,
                Math.toRadians(START_HEADING_DEG)
        );

        follower.setPose(startPose);
    }

    @Override
    public void loop() {

        follower.update();

        Pose robotPose = follower.pose();

        double robotX = robotPose.x();
        double robotY = robotPose.y();
        double robotHeadingRad = robotPose.heading();
        double robotHeadingDeg = normalize360(Math.toDegrees(robotHeadingRad));

        double hiveToRobotX = robotX - HIVE_X;
        double hiveToRobotY = robotY - HIVE_Y;

        double distanceToHive = Math.hypot(hiveToRobotX, hiveToRobotY);

        double angleFromHive = normalize360(
                Math.toDegrees(Math.atan2(hiveToRobotY, hiveToRobotX))
        );

        double sectorError = normalize180(
                angleFromHive - HIVE_OPEN_HEADING_DEG
        );

        boolean modelValid =
                distanceToHive >= MODEL_MIN_DISTANCE
                        && distanceToHive <= MODEL_MAX_DISTANCE
                        && sectorError >= MODEL_MIN_SECTOR
                        && sectorError <= MODEL_MAX_SECTOR;

        double recommendedHeadingDeg = getRecommendedHeadingDeg(
                robotX,
                robotY,
                distanceToHive,
                sectorError
        );

        double headingErrorDeg = normalize180(
                recommendedHeadingDeg - robotHeadingDeg
        );

        boolean headingReady =
                Math.abs(headingErrorDeg) <= HEADING_TOLERANCE_DEG;

        double recommendedRPM = getRecommendedRPM(
                distanceToHive,
                sectorError
        );

        TARGET_RPM = modelValid ? recommendedRPM : 3750.0;

        boolean ballAtF = robot.disF.getState();
        boolean ballAtM = robot.disM.getState();
        boolean ballAtB = robot.disB.getState();
        boolean ballsFull = ballAtF && ballAtM && ballAtB;

        if (ballsFull) {
            robot.ledL.setPosition(LED_DEEP_BLUE);
            robot.ledR.setPosition(LED_DEEP_BLUE);
        } else {
            robot.ledL.setPosition(0.0);
            robot.ledR.setPosition(0.0);
        }

        boolean fireRequested = gamepad1.right_trigger > 0.1;
        boolean autoAimRequested = gamepad1.right_bumper || fireRequested;

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        if (gamepad1.back && !lastFieldCentricButton) {
            fieldCentric = !fieldCentric;
        }
        lastFieldCentricButton = gamepad1.back;

        if (autoAimRequested) {
            if (headingReady) {
                rotate = 0.0;
            } else {
                double aimPower = AIM_KP * Math.abs(headingErrorDeg);
                aimPower = Math.min(aimPower, AIM_MAX_POWER);
                aimPower = Math.max(aimPower, AIM_MIN_POWER);

                rotate = -Math.copySign(aimPower, headingErrorDeg);
            }
        }

        if (fieldCentric) {
            drive.driveFieldCentric(
                    forward,
                    strafe,
                    rotate,
                    robotHeadingRad
            );
        } else {
            drive.driveRobotCentric(
                    forward,
                    strafe,
                    rotate
            );
        }

        double currentRPM = robot.getShooterRPM();
        double error = TARGET_RPM - currentRPM;

        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1_000_000_000.0;
        lastTime = currentTime;

        if (dt <= 0 || dt > 0.2) {
            dt = 0.02;
        }

        integral += error * dt;

        if (kI != 0.0) {
            double maxIntegral = MAX_INTEGRAL_OUTPUT / Math.abs(kI);
            integral = Math.max(
                    -maxIntegral,
                    Math.min(maxIntegral, integral)
            );
        } else {
            integral = 0.0;
        }

        double derivative = (error - lastError) / dt;
        lastError = error;

        double fOutput = kF * TARGET_RPM;
        double pOutput = kP * error;
        double iOutput = kI * integral;
        double dOutput = kD * derivative;

        double rawShooterPower =
                fOutput + pOutput + iOutput + dOutput;

        double batteryVoltage = voltageSensor.getVoltage();

        if (batteryVoltage < 1.0) {
            batteryVoltage = NOMINAL_VOLTAGE;
        }

        double voltageCompensation = NOMINAL_VOLTAGE / batteryVoltage;

        double shooterPower =
                rawShooterPower * voltageCompensation;

        shooterPower = Math.max(
                0.0,
                Math.min(1.0, shooterPower)
        );

        robot.shooter1.setPower(shooterPower);
        robot.shooter2.setPower(shooterPower);

        boolean shooterReady =
                Math.abs(error) <= RPM_TOLERANCE;

        if (gamepad1.left_bumper) {
            robot.intake.setPower(-1.0);
            robot.block.setPosition(RobotHardware.BLOCK_CLOSE);
            feedUnlocked = false;

        } else if (fireRequested) {

            if (!feedUnlocked
                    && modelValid
                    && headingReady
                    && shooterReady) {

                feedUnlocked = true;
            }

            if (feedUnlocked) {
                robot.block.setPosition(RobotHardware.BLOCK_OPEN);
                robot.intake.setPower(FEED_POWER);
            } else {
                robot.block.setPosition(RobotHardware.BLOCK_CLOSE);
                robot.intake.setPower(0.0);
            }

        } else if (gamepad1.left_trigger > 0.1) {
            feedUnlocked = false;
            robot.block.setPosition(RobotHardware.BLOCK_CLOSE);
            robot.intake.setPower(1.0);

        } else {
            feedUnlocked = false;
            robot.intake.setPower(0.0);
            robot.block.setPosition(RobotHardware.BLOCK_CLOSE);
        }

        telemetry.addLine("========== AUTO AIM MODEL V2.1 ==========");
        telemetry.addData("Robot X", "%.2f", robotX);
        telemetry.addData("Robot Y", "%.2f", robotY);
        telemetry.addData("Distance", "%.2f in", distanceToHive);
        telemetry.addData("Sector Error", "%+.1f°", sectorError);
        telemetry.addData("Model Valid", modelValid ? "✓ YES" : "× OUTSIDE");

        telemetry.addLine("");
        telemetry.addData("Heading", "%.1f°", robotHeadingDeg);
        telemetry.addData("Recommended Heading", "%.1f°", recommendedHeadingDeg);
        double geometricHeadingForDisplay =
                normalize360(
                        Math.toDegrees(
                                Math.atan2(
                                        HIVE_Y - robotY,
                                        HIVE_X - robotX
                                )
                        )
                );

        double headingCorrectionForDisplay =
                normalize180(
                        recommendedHeadingDeg
                                - geometricHeadingForDisplay
                );

        telemetry.addData(
                "Heading Correction",
                "%+.1f°",
                headingCorrectionForDisplay
        );
        telemetry.addData("Heading Error", "%+.1f°", headingErrorDeg);
        telemetry.addData("Heading Ready", headingReady ? "✓" : "×");

        telemetry.addLine("");
        telemetry.addData("Recommended RPM", "%.0f", recommendedRPM);
        telemetry.addData("Target RPM", "%.0f", TARGET_RPM);
        telemetry.addData("Actual RPM", "%.0f", currentRPM);
        telemetry.addData("RPM Error", "%.0f", error);
        telemetry.addData("RPM Ready", shooterReady ? "✓" : "×");
        telemetry.addData("Feed Unlocked", feedUnlocked ? "YES" : "NO");

        telemetry.addLine("");
        telemetry.addData("Auto Aim", autoAimRequested ? "ON" : "OFF");
        telemetry.addData("Battery", "%.2f V", batteryVoltage);

        telemetry.addLine("");
        telemetry.addLine("RB = Auto Aim Only");
        telemetry.addLine("RT = Auto Aim + Fire");
        telemetry.addLine("Model outside = RT will NOT feed");

        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("RobotX", robotX);
        packet.put("RobotY", robotY);
        packet.put("DistanceToHive", distanceToHive);
        packet.put("SectorError", sectorError);
        packet.put("ModelValid", modelValid ? 1 : 0);

        packet.put("RobotHeading", robotHeadingDeg);
        packet.put("RecommendedHeading", recommendedHeadingDeg);
        packet.put(
                "HeadingCorrection",
                headingCorrectionForDisplay
        );
        packet.put("HeadingError", headingErrorDeg);
        packet.put("HeadingReady", headingReady ? 1 : 0);

        packet.put("RecommendedRPM", recommendedRPM);
        packet.put("TargetRPM", TARGET_RPM);
        packet.put("ActualRPM", currentRPM);
        packet.put("RPMReady", shooterReady ? 1 : 0);

        packet.put("AutoAim", autoAimRequested ? 1 : 0);
        packet.put("Feed", feedUnlocked ? 1 : 0);
        packet.put("ShooterPower", shooterPower);
        packet.put("BatteryVoltage", batteryVoltage);
        packet.put("AimRotatePower", rotate);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    private double getRecommendedHeadingDeg(
            double robotX,
            double robotY,
            double distanceToHive,
            double sectorError
    ) {

        double geometricHeading =
                Math.toDegrees(
                        Math.atan2(
                                HIVE_Y - robotY,
                                HIVE_X - robotX
                        )
                );

        /*
         * V2.1 左侧 Heading 修正
         *
         * sector >= -40°：
         * 保持原来的几何朝向，不动之前已经比较准的区域。
         *
         * -50° <= sector < -40°：
         * 从 0° 平滑增加到 +4°。
         *
         * + correction = Heading 数值变大
         * = 少顺时针转一点。
         */
        double headingCorrection = 0.0;

        if (sectorError < -40.0) {

            double t =
                    (-sectorError - 40.0)
                            / 10.0;

            t =
                    Math.max(
                            0.0,
                            Math.min(
                                    1.0,
                                    t
                            )
                    );

            headingCorrection =
                    12.0 * t;
        }

        return normalize360(
                geometricHeading
                        + headingCorrection
        );
    }

    private double getRecommendedRPM(
            double distanceToHive,
            double sectorError
    ) {

        if (sectorError <= -25.0) {
            return 3800.0;
        } else if (sectorError <= -15.0) {
            return 3775.0;
        } else if (sectorError <= 10.0) {
            return 3750.0;
        } else if (sectorError <= 20.0) {
            return 3725.0;
        } else if (sectorError <= 30.0) {
            return 3700.0;
        } else {
            return 3675.0;
        }
    }

    private void resetPID() {
        integral = 0.0;
        lastError = 0.0;
        lastTime = System.nanoTime();
    }

    private double normalize360(double angle) {
        angle %= 360.0;

        if (angle < 0) {
            angle += 360.0;
        }

        return angle;
    }

    private double normalize180(double angle) {
        angle %= 360.0;

        if (angle > 180.0) {
            angle -= 360.0;
        }

        if (angle < -180.0) {
            angle += 360.0;
        }

        return angle;
    }

    @Override
    public void stop() {
        drive.stop();

        robot.intake.setPower(0.0);
        robot.shooter1.setPower(0.0);
        robot.shooter2.setPower(0.0);

        robot.block.setPosition(RobotHardware.BLOCK_CLOSE);

        robot.ledL.setPosition(0.0);
        robot.ledR.setPosition(0.0);

        feedUnlocked = false;
    }
}
