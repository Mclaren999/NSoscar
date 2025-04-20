package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.commandbase.Deposit.*;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.*;

import com.pedropathing.localization.Pose;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commandbase.Deposit;
import org.firstinspires.ftc.teamcode.commandbase.Intake;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetDeposit;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetIntake;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetAuto;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.TelemetryData;

@TeleOp(name = "Test ReIntake TeleOp", group = "Test")
public class TestReIntakeTeleOp extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private GamepadEx driver;
    private ElapsedTime timer;
    private TelemetryData telemetryData;

    @Override
    public void initialize() {
        // Set opMode type and initial deposit state
        opModeType = OpModeType.TELEOP;
        depositInit = DepositPivotState.MIDDLE_HOLD;

        // Reset command scheduler
        super.reset();

        // Initialize robot and subsystems
        robot.init(hardwareMap);
        register(robot.deposit, robot.intake);

        // Initialize gamepad and telemetry
        driver = new GamepadEx(gamepad1);
        timer = new ElapsedTime();
        telemetryData = new TelemetryData(telemetry);

        // Mock follower pose for testing (bypass pathing)
        robot.follower.setStartingPose(new Pose(0, 105.125, Math.toRadians(270)));

        // Button bindings for testing
        driver.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> telemetry.addData("Test", "Starting intakeSampleCycleHalf")),
                        intakeSampleCycleHalf(0, 370) // Simulate intake and transfer
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> telemetry.addData("Test", "Starting scoreSampleCycleHalf")),
                        scoreSampleCycleHalf(0) // Simulate scoring with ReIntake check
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> telemetry.addData("Test", "Starting checkTransferAndRetry")),
                        checkTransferAndRetry() // Test ReIntake logic independently
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new InstantCommand(() -> robot.intake.setActiveIntake(IntakeMotorState.STOP)) // Stop intake for manual control
        );

        // Manual deposit control for setup
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new SetDeposit(robot, DepositPivotState.MIDDLE_HOLD, 0, true).withTimeout(600)
        );

        super.run();
    }

    private SequentialCommandGroup intakeSampleCycleHalf(int pathNum, int extendoTarget) {
        // Timing variables (in milliseconds)
        final int INTAKE_STABILIZE = 150;
        final int DEPOSIT_START_DELAY = 200;
        final int DEPOSIT_TO_REACH_MIDDLE_HOLD = 600;
        final int SAMPLE_DETECTION = 1000;
        final int SAMPLE_SETTLE_IN_INTAKE = 100;
        final int INTAKE_GRIP = 100;
        final int INTAKE_TO_REACH_TRANSFER = 400;
        final int DEPOSIT_TO_REACH_TRANSFER = 300;
        final int DEPOSIT_STABILIZE = 200;
        final int CLAW_CLOSE = 300;

        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        // Mock path following with a wait to simulate movement
                        new WaitCommand(500), // Simulate path following delay
                        new SetIntake(robot, Intake.IntakePivotState.INTAKE, IntakeMotorState.FORWARD, 120, true),
                        new SequentialCommandGroup(
                                new WaitCommand(DEPOSIT_START_DELAY), // Delay to allow intake to start
                                new SetDeposit(robot, Deposit.DepositPivotState.MIDDLE_HOLD, 0, true)
                                        .withTimeout(DEPOSIT_TO_REACH_MIDDLE_HOLD) // Time to reach MIDDLE_HOLD
                                        .beforeStarting(new WaitCommand(INTAKE_STABILIZE)) // Time to stabilize intake
                        )
                ),

                new SetIntake(robot, Intake.IntakePivotState.INTAKE, IntakeMotorState.FORWARD, extendoTarget, true),

                new ParallelRaceGroup(
                        new WaitUntilCommand(robot.intake::hasSample)
                ).withTimeout(SAMPLE_DETECTION), // Time to detect a sample

                new WaitCommand(SAMPLE_SETTLE_IN_INTAKE), // Time for sample to settle in intake
                new InstantCommand(() -> robot.intake.setActiveIntake(IntakeMotorState.HOLD)),
                new WaitCommand(INTAKE_GRIP), // Time for intake to grip sample
                new SetIntake(robot, Intake.IntakePivotState.TRANSFER, Intake.IntakeMotorState.HOLD, 0, true),
                new WaitCommand(INTAKE_TO_REACH_TRANSFER), // Time to reach TRANSFER position

                new SetDeposit(robot, Deposit.DepositPivotState.TRANSFER, 0, true).withTimeout(DEPOSIT_TO_REACH_TRANSFER), // Time to reach TRANSFER
                new WaitCommand(DEPOSIT_STABILIZE), // Time for deposit to stabilize
                new InstantCommand(() -> robot.deposit.setClawOpen(false)),
                new WaitCommand(CLAW_CLOSE) // Time for claw to close and sample to settle
        );
    }

    private SequentialCommandGroup checkTransferAndRetry() {
        // Timing variables (in milliseconds)
        final int REINTAKE_ACTIVATE = 500;
        final int INTAKE_TO_REACH_TRANSFER = 400;
        final int DEPOSIT_TO_REACH_TRANSFER = 300;
        final int DEPOSIT_STABILIZE = 200;
        final int CLAW_CLOSE = 300;
        final int REINTAKE_TOTAL = 2000;
        final int EJECT_SAMPLE = 500;
        final int POST_EJECT = 200;

        return new SequentialCommandGroup(
                new RunCommand(() -> {
                    int localRetryCount = 0; // Local retry counter
                    while (robot.intake.hasSample() && localRetryCount < 2) {
                        Command reIntakeSequence = new SequentialCommandGroup(
                                new SetIntake(robot, Intake.IntakePivotState.INTAKE, IntakeMotorState.FORWARD, 0, false)
                                        .withTimeout(REINTAKE_ACTIVATE), // Time to re-engage sample
                                new SetIntake(robot, Intake.IntakePivotState.TRANSFER, Intake.IntakeMotorState.HOLD, 0, true),
                                new WaitCommand(INTAKE_TO_REACH_TRANSFER), // Time to reach TRANSFER position
                                new SetDeposit(robot, Deposit.DepositPivotState.TRANSFER, 0, true)
                                        .withTimeout(DEPOSIT_TO_REACH_TRANSFER), // Time to reach TRANSFER
                                new WaitCommand(DEPOSIT_STABILIZE), // Time for deposit to stabilize
                                new InstantCommand(() -> robot.deposit.setClawOpen(false)),
                                new WaitCommand(CLAW_CLOSE), // Time for claw to close and sample to settle
                                new SetDeposit(robot, DepositPivotState.MIDDLE_HOLD, 0, false)
                                .withTimeout(DEPOSIT_TO_REACH_TRANSFER) // Time to reach TRANSFER
                        );
                        reIntakeSequence.execute();
                        while (!reIntakeSequence.isFinished()) {
                            reIntakeSequence.execute();
                        }
                        reIntakeSequence.end(false);
                        localRetryCount++;
                        telemetry.addData("ReIntake Retries", localRetryCount);
                    }
                }).withTimeout(REINTAKE_TOTAL), // Total time for retry process

                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new SetIntake(robot, Intake.IntakePivotState.INTAKE, IntakeMotorState.REVERSE, 0, false)
                                        .withTimeout(EJECT_SAMPLE), // Time to eject stuck sample
                                new WaitCommand(POST_EJECT), // Time for sample to clear intake
                                new InstantCommand(() -> telemetry.addData("ReIntake Error", "Sample stuck after retries"))
                        ),
                        new InstantCommand(() -> {}),
                        robot.intake::hasSample
                )
        );
    }

    private SequentialCommandGroup scoreSampleCycleHalf(int pathNum) {
        // Timing variables (in milliseconds)
        final int DEPOSIT_TO_REACH_SCORING = 1000;
        final int PRE_SCORE_STABILIZE = 50;
        final int CLAW_OPEN = 200;

        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        checkTransferAndRetry(), // Check transfer success
                        new SetAuto(robot, Deposit.DepositPivotState.SCORING, HIGH_BUCKET_HEIGHT, false)
                                .withTimeout(DEPOSIT_TO_REACH_SCORING) // Time to reach SCORING and extend slides
                ),
                // Mock path following with a wait to simulate movement
                new WaitCommand(500), // Simulate path following delay
                new WaitCommand(PRE_SCORE_STABILIZE), // Time to stabilize before opening claw
                new InstantCommand(() -> robot.deposit.setClawOpen(true)),
                new WaitCommand(CLAW_OPEN) // Time for claw to open and release sample
        );
    }

    @Override
    public void run() {
        // Initialize timer if not set
        if (timer == null) {
            timer = new ElapsedTime();
        }

        // Run command scheduler
        super.run();

        // Telemetry for debugging
        telemetryData.addData("timer", timer.milliseconds());
        telemetryData.addData("hasSample()", robot.intake.hasSample());
        telemetryData.addData("colorSensor getDistance", robot.colorSensor.getDistance(DistanceUnit.CM));
        telemetryData.addData("intakePivotState", intakePivotState);
        telemetryData.addData("depositPivotState", depositPivotState);
        telemetryData.addData("extendo target", robot.intake.target);
        telemetryData.addData("extendo Pos", robot.intake.getExtendoScaledPosition());
        telemetryData.addData("slides target", robot.deposit.target);
        telemetryData.addData("slides Pos", robot.deposit.getLiftScaledPosition());
        telemetryData.update();

        // Clear bulk cache
        robot.ControlHub.clearBulkCache();
    }

    @Override
    public void end() {
        // Optional: Save final pose or cleanup
    }
}