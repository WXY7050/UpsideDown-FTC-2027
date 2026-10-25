package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Constants {


    // ============================================================
    // Drivetrain
    // ============================================================

    public static MecanumConfig drivetrainConfig =
            new MecanumConfig(c -> {

                // ------------------------------------------------
                // Motor Hardware Map 名称
                // ------------------------------------------------

                c.frontLeftName.set("leftFront");
                c.frontRightName.set("rightFront");

                c.backLeftName.set("leftBack");
                c.backRightName.set("rightBack");


                // ------------------------------------------------
                // Motor 方向
                // ------------------------------------------------

                c.frontLeftDirection.set(
                        DcMotorSimple.Direction.REVERSE
                );

                c.frontRightDirection.set(
                        DcMotorSimple.Direction.FORWARD
                );

                c.backLeftDirection.set(
                        DcMotorSimple.Direction.REVERSE
                );

                c.backRightDirection.set(
                        DcMotorSimple.Direction.FORWARD
                );


                // ------------------------------------------------
                // Manual Brake
                // ------------------------------------------------

                c.manualBrakeMode.set(true);
            });


    // ============================================================
    // Pinpoint Localizer
    // ============================================================

    public static PinpointConfig localizerConfig =
            new PinpointConfig(c -> {

                // ------------------------------------------------
                // Hardware Map 名称
                // ------------------------------------------------

                c.name.set("odo");


                // ------------------------------------------------
                // goBILDA Pod 类型
                // ------------------------------------------------

                c.podType.set(
                        GoBildaPinpointDriver
                                .GoBildaOdometryPods
                                .goBILDA_4_BAR_POD
                );


                // ------------------------------------------------
                // Odometry Pod Offset
                //
                // Robot 坐标：
                //
                //          +X Forward
                //              ↑
                //
                //      +Y  ← Robot →  -Y
                //
                //              ↓
                //             -X
                //
                // Forward Pod 在机器人左侧：
                // X = +2.835"
                //
                // Strafe Pod 在机器人后方：
                // Y = -5.354"
                //
                // ------------------------------------------------

                c.xPodOffset.set(2.835);

                c.yPodOffset.set(-5.354);


                // ------------------------------------------------
                // Encoder Direction
                // ------------------------------------------------

                c.xPodDirection.set(
                        GoBildaPinpointDriver
                                .EncoderDirection
                                .FORWARD
                );

                c.yPodDirection.set(
                        GoBildaPinpointDriver
                                .EncoderDirection
                                .REVERSED
                );


                // ------------------------------------------------
                // Units
                // ------------------------------------------------

                c.globalDistanceUnit.set(
                        DistanceUnit.INCH
                );

                c.offsetUnits.set(
                        DistanceUnit.INCH
                );
            });


    // ============================================================
    // Foresight
    // ============================================================

    public static ForesightConfig foresightConfig =
            new ForesightConfig(c -> {

                // =================================================
                // Translational Controller
                // =================================================

                Controller primaryTranslationalForward =
                        Controller.proportional(
                                0.2174180813461496
                        );

                Controller secondaryTranslationalForward =
                        Controller.proportional(
                                0.08033017153927326
                        );


                Controller primaryTranslationalLateral =
                        Controller.proportional(
                                0.2983835697074562
                        );

                Controller secondaryTranslationalLateral =
                        Controller.proportional(
                                0.11024475605108246
                        );


                // -------------------------------------------------
                // Forward Translation
                // -------------------------------------------------

                c.forwardTranslational.set(
                        Controller
                                .piecewise(
                                        secondaryTranslationalForward
                                )
                                .put(
                                        2.5,
                                        primaryTranslationalForward
                                )
                );


                // -------------------------------------------------
                // Strafe Translation
                // -------------------------------------------------

                c.strafeTranslational.set(
                        Controller
                                .piecewise(
                                        secondaryTranslationalLateral
                                )
                                .put(
                                        2.5,
                                        primaryTranslationalLateral
                                )
                );


                // =================================================
                // Coast / Brake
                // =================================================

                c.coast.set(
                        Controller.proportionalFeedforward(
                                0.01230864528500816
                        )
                );

                c.brake.set(
                        Controller.proportionalFeedforward(
                                0.010462348492256935
                        )
                );


                // =================================================
                // Heading
                // =================================================

                c.headingFeedback.set(
                        Controller.proportional(
                                3.589868086245977
                        )
                );


                c.headingBrakeCoefficients.set(
                        Vector2D.cartesian(
                                0.06126448005585089,
                                0.002707077430839763
                        )
                );


                // =================================================
                // Translational Brake Coefficients
                // =================================================

                c.linearBrakeCoefficients.set(
                        Matrix.diag(
                                0.11662015805743567,
                                0.05596690348501773
                        )
                );


                c.quadraticBrakeCoefficients.set(
                        Matrix.diag(
                                8.273717649780762E-4,
                                0.0019339371338567828
                        )
                );


                // =================================================
                // Velocity
                // =================================================

                c.maxAchievableForwardVelocity.set(
                        55.88177046968903
                );

                c.maxAchievableStrafeVelocity.set(
                        67.43035618796497
                );


                // =================================================
                // Natural Deceleration
                // =================================================

                c.naturalForwardDeceleration.set(
                        31.834288056345965
                );

                c.naturalStrafeDeceleration.set(
                        47.74610628733895
                );


                // =================================================
                // End Constraints
                //
                // 默认终点条件很严：
                // translation ≈ 0.1"
                // heading ≈ 0.4°
                // velocity ≈ 0.1 in/s
                //
                // 这里稍微放宽，减少到点后持续来回纠正、抖动。
                // =================================================

                c.translationalConstraint.set(
                        0.50
                );

                c.headingConstraint.set(
                        Math.toRadians(1.5)
                );

                c.velocityConstraint.set(
                        1.0
                );

                // 单位：ms
                // 到 parametric end 后最多再纠正 150ms。
                c.timeoutConstraint.set(
                        150.0
                );
            });


    // ============================================================
    // Create Follower
    // ============================================================

    public static Follower create(HardwareMap hardwareMap) {

        return new Follower(

                // Localization
                new PinpointLocalizer(
                        hardwareMap,
                        localizerConfig
                ),

                // Drivetrain
                new Mecanum(
                        hardwareMap,
                        drivetrainConfig
                ),

                // Path Following / Foresight
                new Foresight(
                        foresightConfig
                )
        );
    }
}