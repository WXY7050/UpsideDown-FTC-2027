package org.firstinspires.ftc.teamcode.upsidedown.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class DriveSubsystem {
    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;

    public DriveSubsystem(HardwareMap hardwareMap) {
        // 电机名称和方向都从PedroPathing的底盘配置中获取，保持与Pedro标定一致
        leftFront = hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.frontLeftName.get());
        leftBack = hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.backLeftName.get());
        rightFront = hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.frontRightName.get());
        rightBack = hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.backRightName.get());

        leftFront.setDirection(Constants.drivetrainConfig.frontLeftDirection.get());
        leftBack.setDirection(Constants.drivetrainConfig.backLeftDirection.get());
        rightFront.setDirection(Constants.drivetrainConfig.frontRightDirection.get());
        rightBack.setDirection(Constants.drivetrainConfig.backRightDirection.get());

        // 设置零功率行为
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 运行模式
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void driveFieldCentric(double forward, double strafe, double rotate, double heading) {
        double rotX = strafe * Math.cos(-heading) - forward * Math.sin(-heading);
        double rotY = strafe * Math.sin(-heading) + forward * Math.cos(-heading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotate), 1);
        double leftFrontPower = (rotY + rotX + rotate) / denominator;
        double leftBackPower = (rotY - rotX + rotate) / denominator;
        double rightFrontPower = (rotY - rotX - rotate) / denominator;
        double rightBackPower = (rotY + rotX - rotate) / denominator;

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }

    public void driveRobotCentric(double forward, double strafe, double rotate) {
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);
        double leftFrontPower = (forward + strafe + rotate) / denominator;
        double leftBackPower = (forward - strafe + rotate) / denominator;
        double rightFrontPower = (forward - strafe - rotate) / denominator;
        double rightBackPower = (forward + strafe - rotate) / denominator;

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }

    public void stop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}