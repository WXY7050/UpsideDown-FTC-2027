package org.firstinspires.ftc.teamcode.upsidedown.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.upsidedown.RobotConstants;
import org.firstinspires.ftc.teamcode.upsidedown.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.upsidedown.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.upsidedown.subsystems.FlowerSubsystem;
import org.firstinspires.ftc.teamcode.upsidedown.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.upsidedown.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.upsidedown.subsystems.ShooterSubsystem;


@Config
@TeleOp(name = "UpsideDown 手动", group = "UpsideDown")
public class UpsideDownTeleOpMain extends OpMode {

    // =========================================================
    // Hardware / Pedro / Subsystems
    // =========================================================

    private RobotHardware robot;
    private Follower follower;

    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private FlowerSubsystem flower;
    private LEDSubsystem led;

    private VoltageSensor voltageSensor;


    // =========================================================
    // 临时 TeleOp 起始 Pose
    //
    // 后续我们会做 Auto -> TeleOp Pose 无缝衔接，
    // 到时这里不再作为比赛时最终 Pose 来源。
    // =========================================================

    public static double START_X = 12.441 / 2.0;
    public static double START_Y = 12.835 / 2.0;
    public static double START_HEADING_DEG = 90.0;


    // =========================================================
    // Alliance
    //
    // true  = RED
    // false = BLUE
    // =========================================================

    public static boolean RED_ALLIANCE = true;


    // =========================================================
    // 全场 HIVE 坐标
    //
    // RED：左列
    // BLUE：右列
    //
    // 下排开口 270°
    // 上排开口 90°
    // =========================================================

    public static double RED_LOWER_X = 57.0;
    public static double RED_LOWER_Y = 55.0;
    public static double RED_LOWER_OPEN_HEADING = 270.0;

    public static double RED_UPPER_X = 57.0;
    public static double RED_UPPER_Y = 86.5;
    public static double RED_UPPER_OPEN_HEADING = 90.0;

    public static double BLUE_LOWER_X = 82.5;
    public static double BLUE_LOWER_Y = 55.0;
    public static double BLUE_LOWER_OPEN_HEADING = 270.0;

    public static double BLUE_UPPER_X = 82.5;
    public static double BLUE_UPPER_Y = 86.5;
    public static double BLUE_UPPER_OPEN_HEADING = 90.0;


    // =========================================================
    // Auto Aim Model 范围
    // =========================================================

    public static double MODEL_MIN_DISTANCE = 20.0;
    public static double MODEL_MAX_DISTANCE = 90.0;

    public static double MODEL_MIN_SECTOR = -50.0;
    public static double MODEL_MAX_SECTOR = 40.0;


    // =========================================================
    // Auto Aim Heading 控制
    // =========================================================

    public static double HEADING_TOLERANCE_DEG = 1.5;

    public static double AIM_KP = 0.018;
    public static double AIM_MAX_POWER = 0.55;
    public static double AIM_MIN_POWER = 0.07;


    // =========================================================
    // Shooter Auto Aim Model
    //
    // Shooter PIDF / Tolerance 等底层参数统一由
    // RobotConstants + ShooterSubsystem 管理。
    // =========================================================

    public static double DEFAULT_TARGET_RPM = 3750.0;

    // 距离修正：
    // targetRPM =
    // sectorBaseRPM + (distance - RPM_DISTANCE_BASE) * RPM_PER_INCH
    public static double RPM_DISTANCE_BASE = 48.0;
    public static double RPM_PER_INCH = 7.0;


    // =========================================================
    // Driver Control State
    // =========================================================

    private boolean fieldCentric = true;
    private boolean lastFieldCentricButton = false;

    /*
     * false:
     * 本次 RT 还没有同时满足
     * Model Valid + Heading Ready + Shooter Ready
     *
     * true:
     * 已经解锁。
     * 只要 RT 继续按住，持续 Feed，
     * 不再因为 RPM 暂时下降而停顿。
     */
    private boolean feedUnlocked = false;


    // =========================================================
    // INIT
    // =========================================================

    @Override
    public void init() {
        // Hardware
        robot = new RobotHardware();
        robot.init(hardwareMap);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Pedro
        follower = Constants.create(hardwareMap);

        // Subsystems
        drive = new DriveSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(robot, voltageSensor);
        intake = new IntakeSubsystem(robot);
        flower = new FlowerSubsystem(robot);
        led = new LEDSubsystem(robot);

        // Shooter 参数
        shooter.setRpmTolerance(RobotConstants.SHOOTER_RPM_TOLERANCE);
        shooter.setTargetRPM(DEFAULT_TARGET_RPM);

        // 初始机构状态
        shooter.closeBlock();
        intake.stop();
        flower.down();
        led.showDefault();

        telemetry.addData("状态", "UpsideDown 模块化 TeleOp 初始化完成");
        telemetry.addData("Alliance", RED_ALLIANCE ? "RED" : "BLUE");
        telemetry.addData("RPM Tolerance", "%.0f", RobotConstants.SHOOTER_RPM_TOLERANCE);
        telemetry.addData("Feed Power", "%.2f", RobotConstants.FEED_POWER);
        telemetry.addLine("");
        telemetry.addLine("RB = Auto Aim");
        telemetry.addLine("RT = Auto Aim + Fire");
        telemetry.update();
    }


    // =========================================================
    // START
    // =========================================================

    @Override
    public void start() {
        feedUnlocked = false;
        shooter.resetPID();
        Pose startPose = new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
        follower.setPose(startPose);
    }


    // =========================================================
    // LOOP
    // =========================================================

    @Override
    public void loop() {

        // =====================================================
        // Pedro Localization
        // =====================================================
        follower.update();
        Pose robotPose = follower.pose();
        double robotX = robotPose.x();
        double robotY = robotPose.y();
        double robotHeadingRad = robotPose.heading();
        double robotHeadingDeg = normalize360(Math.toDegrees(robotHeadingRad));


        // =====================================================
        // 选择当前目标 HIVE
        // =====================================================

        HiveTarget lowerHive;
        HiveTarget upperHive;

        if (RED_ALLIANCE) {

            lowerHive =
                    evaluateHive(
                            "RED LOWER",
                            RED_LOWER_X,
                            RED_LOWER_Y,
                            RED_LOWER_OPEN_HEADING,
                            robotX,
                            robotY
                    );

            upperHive =
                    evaluateHive(
                            "RED UPPER",
                            RED_UPPER_X,
                            RED_UPPER_Y,
                            RED_UPPER_OPEN_HEADING,
                            robotX,
                            robotY
                    );

        } else {

            lowerHive =
                    evaluateHive(
                            "BLUE LOWER",
                            BLUE_LOWER_X,
                            BLUE_LOWER_Y,
                            BLUE_LOWER_OPEN_HEADING,
                            robotX,
                            robotY
                    );

            upperHive =
                    evaluateHive(
                            "BLUE UPPER",
                            BLUE_UPPER_X,
                            BLUE_UPPER_Y,
                            BLUE_UPPER_OPEN_HEADING,
                            robotX,
                            robotY
                    );
        }


        HiveTarget targetHive = selectBestHive(lowerHive, upperHive);

        double targetHiveX = targetHive.x;
        double targetHiveY = targetHive.y;
        double targetOpenHeadingDeg = targetHive.openHeadingDeg;
        double distanceToHive = targetHive.distance;
        double sectorError = targetHive.sectorError;

        boolean distanceValid = distanceToHive >= MODEL_MIN_DISTANCE && distanceToHive <= MODEL_MAX_DISTANCE;
        boolean sectorValid = sectorError >= MODEL_MIN_SECTOR && sectorError <= MODEL_MAX_SECTOR;
        boolean modelValid = distanceValid && sectorValid;

        // =====================================================
        // Auto Aim Heading
        // =====================================================
        double recommendedHeadingDeg = getRecommendedHeadingDeg(robotX, robotY, targetHiveX, targetHiveY, sectorError);
        double headingErrorDeg = normalize180(recommendedHeadingDeg - robotHeadingDeg);
        boolean headingReady = Math.abs(headingErrorDeg) <= HEADING_TOLERANCE_DEG;

        // =====================================================
        // Auto Aim RPM Model
        // =====================================================
        double sectorBaseRPM = getSectorBaseRPM(sectorError);
        double distanceRPMCorrection = getDistanceRPMCorrection(distanceToHive);
        double recommendedRPM = getRecommendedRPM(distanceToHive, sectorError);
        double targetRPM = modelValid ? recommendedRPM : DEFAULT_TARGET_RPM;
        shooter.setTargetRPM(targetRPM);

        // =====================================================
        // 三传感器满球检测
        // =====================================================
        boolean ballAtF = robot.disF.getState();
        boolean ballAtM = robot.disM.getState();
        boolean ballAtB = robot.disB.getState();
        boolean ballsFull = ballAtF && ballAtM && ballAtB;

        // =====================================================
        // LED
        // =====================================================
        led.setByBallStatus(ballsFull);


        // =====================================================
        // Driver Inputs
        // =====================================================
        boolean fireRequested = gamepad1.right_trigger > 0.1;
        boolean autoAimRequested = gamepad1.right_bumper || fireRequested;

        double stickForward = -gamepad1.left_stick_y;
        double stickRight = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        // Back: Field Centric / Robot Centric
        if (gamepad1.back && !lastFieldCentricButton) {
            fieldCentric = !fieldCentric;
        }
        lastFieldCentricButton = gamepad1.back;


        // =====================================================
        // Auto Aim Rotation
        // =====================================================
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


        // =====================================================
        // Drive
        // =====================================================
        if (fieldCentric) {
            double fieldForward;
            double fieldStrafe;

            if (RED_ALLIANCE) {
                fieldStrafe = stickForward;
                fieldForward = -stickRight;
            } else {
                fieldStrafe = -stickForward;
                fieldForward = stickRight;
            }

            double driveHeadingRad = robotHeadingRad - Math.PI / 2.0;
            drive.driveFieldCentric(fieldForward, fieldStrafe, rotate, driveHeadingRad);
        } else {
            drive.driveRobotCentric(stickForward, stickRight, rotate);
        }


        // =====================================================
        // Shooter
        // =====================================================
        shooter.update();
        double currentRPM = shooter.getCurrentRPM();
        double rpmError = shooter.getError();
        double shooterPower = shooter.getCurrentPower();
        boolean shooterReady = shooter.isReady();

        // =====================================================
        // Intake / Feed / Block
        // =====================================================
        if (gamepad1.left_bumper) {
            intake.outtake();
            shooter.closeBlock();
            feedUnlocked = false;
        } else if (fireRequested) {
            if (!feedUnlocked && modelValid && headingReady && shooterReady) {
                feedUnlocked = true;
            }

            if (feedUnlocked) {
                shooter.openBlock();
                intake.setIntakePower(RobotConstants.FEED_POWER);
            } else {
                shooter.closeBlock();
                intake.stop();
            }
        } else if (gamepad1.left_trigger > 0.1) {
            feedUnlocked = false;
            shooter.closeBlock();
            intake.intake();
        } else {
            feedUnlocked = false;
            intake.stop();
            shooter.closeBlock();
        }

        // =====================================================
        // Flower
        // =====================================================
        if (gamepad1.dpad_up) {
            flower.up();
        } else if (gamepad1.dpad_down) {
            flower.down();
        }


        // =====================================================
        // Telemetry
        // =====================================================

        double geometricHeadingForDisplay =
                normalize360(
                        Math.toDegrees(
                                Math.atan2(
                                        targetHiveY
                                                - robotY,
                                        targetHiveX
                                                - robotX
                                )
                        )
                );

        double headingCorrectionForDisplay =
                normalize180(
                        recommendedHeadingDeg
                                - geometricHeadingForDisplay
                );

        boolean fireReady =
                modelValid
                        && headingReady
                        && shooterReady;


        telemetry.addLine(
                "========== UPSIDEDOWN AUTO AIM =========="
        );

        telemetry.addData(
                "Alliance",
                RED_ALLIANCE
                        ? "RED"
                        : "BLUE"
        );

        telemetry.addData(
                "Drive Mode",
                fieldCentric
                        ? "FIELD"
                        : "ROBOT"
        );

        telemetry.addData(
                "Driver Forward",
                RED_ALLIANCE
                        ? "+X / 0°"
                        : "-X / 180°"
        );


        telemetry.addLine("");

        telemetry.addData(
                "Target Hive",
                targetHive.name
        );

        telemetry.addData(
                "Target X",
                "%.2f",
                targetHiveX
        );

        telemetry.addData(
                "Target Y",
                "%.2f",
                targetHiveY
        );

        telemetry.addData(
                "Target Open",
                "%.0f°",
                targetOpenHeadingDeg
        );


        telemetry.addLine("");

        telemetry.addData(
                "Robot X",
                "%.2f",
                robotX
        );

        telemetry.addData(
                "Robot Y",
                "%.2f",
                robotY
        );

        telemetry.addData(
                "Distance",
                "%.2f in",
                distanceToHive
        );

        telemetry.addData(
                "Distance Valid",
                distanceValid
                        ? "✓ YES"
                        : "× NO"
        );

        telemetry.addData(
                "Sector Error",
                "%+.1f°",
                sectorError
        );

        telemetry.addData(
                "Sector Valid",
                sectorValid
                        ? "✓ YES"
                        : "× NO"
        );

        telemetry.addData(
                "Model Valid",
                modelValid
                        ? "✓ YES"
                        : "× OUTSIDE"
        );


        telemetry.addLine("");

        telemetry.addData(
                "Heading",
                "%.1f°",
                robotHeadingDeg
        );

        telemetry.addData(
                "Recommended Heading",
                "%.1f°",
                recommendedHeadingDeg
        );

        telemetry.addData(
                "Heading Correction",
                "%+.1f°",
                headingCorrectionForDisplay
        );

        telemetry.addData(
                "Heading Error",
                "%+.1f°",
                headingErrorDeg
        );

        telemetry.addData(
                "Heading Ready",
                headingReady
                        ? "✓"
                        : "×"
        );


        telemetry.addLine("");

        telemetry.addData(
                "Sector Base RPM",
                "%.0f",
                sectorBaseRPM
        );

        telemetry.addData(
                "Distance RPM Correction",
                "%+.0f",
                distanceRPMCorrection
        );

        telemetry.addData(
                "Recommended RPM",
                "%.0f",
                recommendedRPM
        );

        telemetry.addData(
                "Target RPM",
                "%.0f",
                shooter.getTargetRPM()
        );

        telemetry.addData(
                "Actual RPM",
                "%.0f",
                currentRPM
        );

        telemetry.addData(
                "RPM Error",
                "%.0f",
                rpmError
        );

        telemetry.addData(
                "RPM Tolerance",
                "%.0f",
                shooter.getRpmTolerance()
        );

        telemetry.addData(
                "RPM Ready",
                shooterReady
                        ? "✓"
                        : "×"
        );

        telemetry.addData(
                "Fire Ready",
                fireReady
                        ? "✓ READY"
                        : "× WAIT"
        );

        telemetry.addData(
                "Feed Unlocked",
                feedUnlocked
                        ? "YES"
                        : "NO"
        );

        telemetry.addData(
                "Feed Power",
                "%.2f",
                RobotConstants.FEED_POWER
        );


        telemetry.addLine("");

        telemetry.addData(
                "disF",
                ballAtF
                        ? "BALL"
                        : "EMPTY"
        );

        telemetry.addData(
                "disM",
                ballAtM
                        ? "BALL"
                        : "EMPTY"
        );

        telemetry.addData(
                "disB",
                ballAtB
                        ? "BALL"
                        : "EMPTY"
        );

        telemetry.addData(
                "Balls Full",
                ballsFull
                        ? "YES"
                        : "NO"
        );

        telemetry.addData(
                "Flower Position",
                "%.2f",
                flower.getPosition()
        );


        telemetry.addLine("");

        telemetry.addData(
                "Auto Aim",
                autoAimRequested
                        ? "ON"
                        : "OFF"
        );

        telemetry.addData(
                "Shooter Power",
                "%.3f",
                shooterPower
        );

        telemetry.addData(
                "Battery",
                "%.2f V",
                shooter.getBatteryVoltage()
        );


        telemetry.addLine("");

        telemetry.addLine(
                "RB = Auto Aim Only"
        );

        telemetry.addLine(
                "RT = Auto Aim + Fire"
        );

        telemetry.addLine(
                "LT = Intake"
        );

        telemetry.addLine(
                "LB = Reverse"
        );

        telemetry.addLine(
                "D-pad UP/DOWN = Flower"
        );

        telemetry.addLine(
                "Back = Field / Robot Centric"
        );

        telemetry.addLine(
                "Model outside = RT will NOT feed"
        );

        telemetry.update();


        // =====================================================
        // FTC Dashboard
        // =====================================================

        TelemetryPacket packet =
                new TelemetryPacket();

        packet.put(
                "AllianceRed",
                RED_ALLIANCE
                        ? 1
                        : 0
        );

        packet.put(
                "TargetHive",
                targetHive.name
        );

        packet.put(
                "TargetHiveX",
                targetHiveX
        );

        packet.put(
                "TargetHiveY",
                targetHiveY
        );

        packet.put(
                "TargetOpenHeading",
                targetOpenHeadingDeg
        );

        packet.put(
                "RobotX",
                robotX
        );

        packet.put(
                "RobotY",
                robotY
        );

        packet.put(
                "DistanceToHive",
                distanceToHive
        );

        packet.put(
                "DistanceValid",
                distanceValid
                        ? 1
                        : 0
        );

        packet.put(
                "SectorError",
                sectorError
        );

        packet.put(
                "SectorValid",
                sectorValid
                        ? 1
                        : 0
        );

        packet.put(
                "ModelValid",
                modelValid
                        ? 1
                        : 0
        );

        packet.put(
                "RobotHeading",
                robotHeadingDeg
        );

        packet.put(
                "RecommendedHeading",
                recommendedHeadingDeg
        );

        packet.put(
                "HeadingCorrection",
                headingCorrectionForDisplay
        );

        packet.put(
                "HeadingError",
                headingErrorDeg
        );

        packet.put(
                "HeadingReady",
                headingReady
                        ? 1
                        : 0
        );

        packet.put(
                "SectorBaseRPM",
                sectorBaseRPM
        );

        packet.put(
                "DistanceRPMCorrection",
                distanceRPMCorrection
        );

        packet.put(
                "RecommendedRPM",
                recommendedRPM
        );

        packet.put(
                "TargetRPM",
                shooter.getTargetRPM()
        );

        packet.put(
                "ActualRPM",
                currentRPM
        );

        packet.put(
                "RPMError",
                rpmError
        );

        packet.put(
                "RPMReady",
                shooterReady
                        ? 1
                        : 0
        );

        packet.put(
                "FireReady",
                fireReady
                        ? 1
                        : 0
        );

        packet.put(
                "AutoAim",
                autoAimRequested
                        ? 1
                        : 0
        );

        packet.put(
                "Feed",
                feedUnlocked
                        ? 1
                        : 0
        );

        packet.put(
                "FeedPower",
                RobotConstants.FEED_POWER
        );

        packet.put(
                "BallAtF",
                ballAtF
                        ? 1
                        : 0
        );

        packet.put(
                "BallAtM",
                ballAtM
                        ? 1
                        : 0
        );

        packet.put(
                "BallAtB",
                ballAtB
                        ? 1
                        : 0
        );

        packet.put(
                "BallsFull",
                ballsFull
                        ? 1
                        : 0
        );

        packet.put(
                "FlowerPosition",
                flower.getPosition()
        );

        packet.put(
                "ShooterPower",
                shooterPower
        );

        packet.put(
                "BatteryVoltage",
                shooter.getBatteryVoltage()
        );

        packet.put(
                "AimRotatePower",
                rotate
        );


        FtcDashboard
                .getInstance()
                .sendTelemetryPacket(
                        packet
                );
    }


    // =========================================================
    // HIVE Target 数据结构
    // =========================================================

    private static class HiveTarget {

        final String name;
        final double x;
        final double y;
        final double openHeadingDeg;

        final double distance;
        final double sectorError;

        final boolean modelValid;


        HiveTarget(
                String name,
                double x,
                double y,
                double openHeadingDeg,
                double distance,
                double sectorError,
                boolean modelValid
        ) {

            this.name =
                    name;

            this.x =
                    x;

            this.y =
                    y;

            this.openHeadingDeg =
                    openHeadingDeg;

            this.distance =
                    distance;

            this.sectorError =
                    sectorError;

            this.modelValid =
                    modelValid;
        }
    }


    // =========================================================
    // Evaluate HIVE
    // =========================================================

    private HiveTarget evaluateHive(String name, double hiveX, double hiveY, double openHeadingDeg, double robotX, double robotY) {
        double hiveToRobotX = robotX - hiveX;
        double hiveToRobotY = robotY - hiveY;
        double distance = Math.hypot(hiveToRobotX, hiveToRobotY);
        double angleFromHive = normalize360(Math.toDegrees(Math.atan2(hiveToRobotY, hiveToRobotX)));
        double sectorError = normalize180(angleFromHive - openHeadingDeg);
        boolean modelValid = distance >= MODEL_MIN_DISTANCE && distance <= MODEL_MAX_DISTANCE && sectorError >= MODEL_MIN_SECTOR && sectorError <= MODEL_MAX_SECTOR;

        return new HiveTarget(name, hiveX, hiveY, openHeadingDeg, distance, sectorError, modelValid);
    }


    // =========================================================
    // 自动选择目标 HIVE
    // =========================================================

    private HiveTarget selectBestHive(HiveTarget lowerHive, HiveTarget upperHive) {
        if (lowerHive.modelValid && upperHive.modelValid) {
            return lowerHive.distance <= upperHive.distance ? lowerHive : upperHive;
        }

        if (lowerHive.modelValid) { return lowerHive; }
        if (upperHive.modelValid) { return upperHive; }

        return lowerHive.distance <= upperHive.distance ? lowerHive : upperHive;
    }


    // =========================================================
    // 推荐 Heading
    // =========================================================

    private double getRecommendedHeadingDeg(double robotX, double robotY, double hiveX, double hiveY, double sectorError) {
        double geometricHeading = Math.toDegrees(Math.atan2(hiveY - robotY, hiveX - robotX));
        double headingCorrection = 0.0;

        if (sectorError < -40.0) {
            double t = (-sectorError - 40.0) / 10.0;
            t = Math.max(0.0, Math.min(1.0, t));
            headingCorrection = 4.0 * t;
        }

        return normalize360(geometricHeading + headingCorrection);
    }


    // =========================================================
    // Sector 基础 RPM
    // =========================================================

    private double getSectorBaseRPM(double sectorError) {
        if (sectorError <= -25.0) { return 3800.0; }
        else if (sectorError <= -15.0) { return 3775.0; }
        else if (sectorError <= 10.0) { return 3750.0; }
        else if (sectorError <= 20.0) { return 3725.0; }
        else if (sectorError <= 30.0) { return 3700.0; }
        else { return 3675.0; }
    }

    private double getDistanceRPMCorrection(double distanceToHive) {
        return (distanceToHive - RPM_DISTANCE_BASE) * RPM_PER_INCH;
    }

    private double getRecommendedRPM(double distanceToHive, double sectorError) {
        double rpm = getSectorBaseRPM(sectorError) + getDistanceRPMCorrection(distanceToHive);
        return Math.round(rpm / 25.0) * 25.0;
    }


    // =========================================================
    // Angle Helpers
    // =========================================================

    private double normalize360(double angle) {
        angle %= 360.0;
        if (angle < 0.0) { angle += 360.0; }
        return angle;
    }

    private double normalize180(double angle) {
        angle %= 360.0;
        if (angle > 180.0) { angle -= 360.0; }
        if (angle < -180.0) { angle += 360.0; }
        return angle;
    }


    // =========================================================
    // STOP
    // =========================================================

    @Override
    public void stop() {
        drive.stop();
        intake.stop();
        shooter.stop();
        shooter.closeBlock();
        led.off();
        feedUnlocked = false;
    }
}
