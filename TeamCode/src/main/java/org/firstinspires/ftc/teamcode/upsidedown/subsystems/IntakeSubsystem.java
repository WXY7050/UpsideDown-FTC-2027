package org.firstinspires.ftc.teamcode.upsidedown.subsystems;

import static org.firstinspires.ftc.teamcode.upsidedown.hardware.RobotHardware.INTAKE_DOWN;
import static org.firstinspires.ftc.teamcode.upsidedown.hardware.RobotHardware.INTAKE_UP;

import org.firstinspires.ftc.teamcode.upsidedown.hardware.RobotHardware;

public class IntakeSubsystem {
    private RobotHardware hardware;

    // 吸取电机功率常量
    public static final double INTAKE_POWER = 1.0;
    public static final double OUTTAKE_POWER = -1.0;

    public IntakeSubsystem(RobotHardware hardware) {
        this.hardware = hardware;
    }

    public void intake() {
        hardware.intake.setPower(INTAKE_POWER);
    }

    public void outtake() {
        hardware.intake.setPower(OUTTAKE_POWER);
    }

    public void stop() {
        hardware.intake.setPower(0);
    }

    public void setIntakePower(double power) {
        hardware.intake.setPower(power);
    }

}