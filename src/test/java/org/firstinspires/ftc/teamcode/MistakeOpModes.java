package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * The mistakes new programmers make, for the tests to check the sim explains each one.
 * They're test classes, so they never show up on a real run's OpMode list.
 */
public final class MistakeOpModes {

    private MistakeOpModes() {}

    /** Drives forward on every motor without reversing the mirrored left side: it spins instead */
    @TeleOp(name = "Mistake: no reverse", group = "test")
    public static class NoReverse extends OpMode {
        private DcMotor[] wheels;

        @Override
        public void init() {
            wheels = new DcMotor[]{hardwareMap.get(DcMotor.class, "frontLeft"), hardwareMap.get(DcMotor.class, "frontRight"),
                    hardwareMap.get(DcMotor.class, "backLeft"), hardwareMap.get(DcMotor.class, "backRight")};
        }

        @Override
        public void loop() {
            for (DcMotor wheel : wheels) wheel.setPower(-gamepad1.left_stick_y);
        }
    }

    @TeleOp(name = "Mistake: typo", group = "test")
    public static class Typo extends OpMode {
        @Override
        public void init() {
            hardwareMap.get(DcMotor.class, "intak");
        }

        @Override
        public void loop() {}
    }

    @TeleOp(name = "Mistake: never got the motor", group = "test")
    public static class NeverGotMotor extends OpMode {
        private DcMotor intake;

        @Override
        public void init() {}

        @Override
        public void loop() {
            intake.setPower(gamepad1.right_trigger);
        }
    }

    @TeleOp(name = "Mistake: while in loop", group = "test")
    public static class WhileInLoop extends OpMode {
        private DcMotor intake;

        @Override
        public void init() {
            intake = hardwareMap.get(DcMotor.class, "intake");
        }

        @Override
        public void loop() {
            while (gamepad1.right_trigger < 0.5) {
                intake.setPower(0);
            }
            intake.setPower(1);
        }
    }

    @TeleOp(name = "Mistake: ignores stop", group = "test")
    public static class IgnoresStop extends LinearOpMode {
        @Override
        public void runOpMode() {
            DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
            waitForStart();
            while (true) {
                intake.setPower(1);
            }
        }
    }

    @TeleOp(name = "Mistake: moves in init", group = "test")
    public static class MovesInInit extends OpMode {
        @Override
        public void init() {
            hardwareMap.get(DcMotor.class, "intake").setPower(1);
        }

        @Override
        public void loop() {}
    }

    /** Feeds balls into the shooter without ever turning it on */
    @TeleOp(name = "Mistake: no spin up", group = "test")
    public static class NoSpinUp extends OpMode {
        @Override
        public void init() {}

        @Override
        public void loop() {
            hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "gate").setPosition(1);
            hardwareMap.get(DcMotor.class, "intake").setPower(1);
        }
    }

    /** Not a mistake: the old way of getting a motor still works */
    @TeleOp(name = "Old style hardwareMap", group = "test")
    public static class OldStyle extends OpMode {
        private DcMotor intake;

        @Override
        public void init() {
            intake = hardwareMap.dcMotor.get("intake");
        }

        @Override
        public void loop() {
            intake.setPower(1);
        }
    }
}
