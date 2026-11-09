package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * One way to drive the starter bot, for the sim's own tests and for mentors. Students write their own.
 *
 * Left stick drives and strafes, right stick turns. Right trigger runs the intake, B spits a ball out.
 * Hold left trigger to spin up the shooter, then hold A to shoot.
 */
@TeleOp(name = "Starter Bot (answer key)")
public class StarterBotTeleOp extends OpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight, intake, shooter;
    private Servo gate;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        gate = hardwareMap.get(Servo.class, "gate");

        // the left motors are mounted mirrored
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addLine("Ready");
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y; // pushing the stick forward is negative
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;
        double biggest = Math.max(1, Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br))));
        frontLeft.setPower(fl / biggest);
        frontRight.setPower(fr / biggest);
        backLeft.setPower(bl / biggest);
        backRight.setPower(br / biggest);

        boolean shooting = gamepad1.a;
        shooter.setPower(gamepad1.left_trigger > 0.1 || shooting ? 0.8 : 0);
        gate.setPosition(shooting ? 1 : 0);
        if (gamepad1.right_trigger > 0.1 || shooting) {
            intake.setPower(1);
        } else if (gamepad1.b) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }

        telemetry.addData("Shooter", shooting ? "shooting" : "ready");
    }
}
