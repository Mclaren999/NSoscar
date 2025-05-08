package org.firstinspires.ftc.teamcode.opmode.TeleOp.Test;

import static org.firstinspires.ftc.teamcode.commandbase.Deposit.DepositPivotState;
import static org.firstinspires.ftc.teamcode.commandbase.Deposit.depositPivotState;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.IntakeMotorState;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.IntakePivotState;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.SampleColorDetected;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.intakeMotorState;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.intakePivotState;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.sampleColor;
import static org.firstinspires.ftc.teamcode.hardware.Globals.HIGH_BUCKET_HEIGHT;
import static org.firstinspires.ftc.teamcode.hardware.Globals.INTAKE_HOLD_SPEED;
import static org.firstinspires.ftc.teamcode.hardware.Globals.MAX_EXTENDO_EXTENSION;
import static org.firstinspires.ftc.teamcode.hardware.Globals.OpModeType;
import static org.firstinspires.ftc.teamcode.hardware.Globals.autoEndPose;
import static org.firstinspires.ftc.teamcode.hardware.Globals.depositInit;
import static org.firstinspires.ftc.teamcode.hardware.Globals.opModeType;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.UninterruptibleCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commandbase.Deposit;
import org.firstinspires.ftc.teamcode.commandbase.Intake;
import org.firstinspires.ftc.teamcode.commandbase.commands.RealTransfer;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetDeposit;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetIntake;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.TelemetryData;

@TeleOp(name = "ATestTeleOp")
public class TestTeleOp extends CommandOpMode {
    public GamepadEx driver;
    public ElapsedTime timer;
    public ElapsedTime gameTimer;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    private final Robot robot = Robot.getInstance();
    private boolean sampleDetected = false;
    private boolean transferTriggered = false;
    private boolean transferSuccessful = false;

    private SequentialCommandGroup getTransferAndScoreCommand() {
        return new SequentialCommandGroup(
                // Transfer sample
                new RealTransfer(robot),
                new WaitCommand(200),
                // Score sample
                new SetDeposit(robot, DepositPivotState.SCORING, HIGH_BUCKET_HEIGHT, true).withTimeout(1500),
                new WaitCommand(200),
                // Revert to neutral state and reset
                new SetIntake(robot, IntakePivotState.INTAKE_READY, IntakeMotorState.STOP, 0, true),
                new SetDeposit(robot, DepositPivotState.MIDDLE_HOLD, 0, true).withTimeout(1500),
                new InstantCommand(() -> {
                    transferSuccessful = !robot.intake.hasSample();
                    sampleDetected = false;
                    transferTriggered = false;
                    robot.intake.resetSampleDetection();
                    telemetryData.addData("Transfer Result", transferSuccessful ? "Success: Sample transferred" : "Failsafe: Sample still detected");
                })
        );
    }

    @Override
    public void initialize() {
        opModeType = OpModeType.TELEOP;
        depositInit = DepositPivotState.MIDDLE_HOLD;
        INTAKE_HOLD_SPEED = 0;

        super.reset();
        robot.init(hardwareMap);

        register(robot.deposit, robot.intake);
        robot.intake.setActiveIntake(IntakeMotorState.STOP);

        driver = new GamepadEx(gamepad1);

        // Driver Gamepad controls
        driver.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new UninterruptibleCommand(
                        new FollowPathCommand(
                                robot.follower,
                                robot.follower.pathBuilder()
                                        .addPath(
                                                new BezierLine(
                                                        new Point(robot.follower.getPose().getX(), robot.follower.getPose().getY(), Point.CARTESIAN),
                                                        new Point(13, 127.1, Point.CARTESIAN)
                                                )
                                        )
                                        .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(270))
                                        .setReversed(true)
                                        .build()
                        ).setHoldEnd(true)
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new SequentialCommandGroup(
                        new SetIntake(robot, IntakePivotState.INTAKE, IntakeMotorState.FORWARD, MAX_EXTENDO_EXTENSION, true),
                        new InstantCommand(() -> {
                            robot.intake.setActiveIntake(IntakeMotorState.FORWARD);
                            telemetryData.addData("Intake Motor", "Set to FORWARD");
                        })
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new SequentialCommandGroup(
                        new SetIntake(robot, IntakePivotState.INTAKE_READY, IntakeMotorState.STOP, 0, true),
                        new InstantCommand(() -> {
                            sampleDetected = false;
                            transferTriggered = false;
                            transferSuccessful = false;
                            robot.intake.resetSampleDetection();
                            telemetryData.addData("Intake Reset", "Ready for new sample");
                        })
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new UninterruptibleCommand(
                        new SequentialCommandGroup(
                                new SetIntake(robot, IntakePivotState.INTAKE, IntakeMotorState.REVERSE, 0, true),
                                new WaitCommand(1000),
                                new SetIntake(robot, IntakePivotState.INTAKE_READY, IntakeMotorState.STOP, 0, true),
                                new InstantCommand(() -> {
                                    sampleDetected = false;
                                    transferTriggered = false;
                                    transferSuccessful = false;
                                    robot.intake.resetSampleDetection();
                                    telemetryData.addData("Eject Reset", "Intake cleared, ready for new sample");
                                })
                        )
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new UninterruptibleCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.deposit.setClawOpen(true)),
                                new WaitCommand(300),
                                new SetDeposit(robot, DepositPivotState.MIDDLE_HOLD, 0, true).withTimeout(2000)
                        )
                )
        );

        super.run();
    }

    @Override
    public void run() {
        super.run();
        if (timer == null) {
            robot.initHasMovement();
            INTAKE_HOLD_SPEED = 0.15;
            timer = new ElapsedTime();
            gameTimer = new ElapsedTime();
        }

        if (robot.intake.hasSample() && !sampleDetected && !transferTriggered) {
            sampleDetected = true;
            transferTriggered = true;
            telemetryData.addData("Transfer Triggered", "Sample detected, starting transfer");
            new UninterruptibleCommand(
                    getTransferAndScoreCommand()
            ).schedule();
        }

        if (sampleColor.equals(SampleColorDetected.RED)) {
            gamepad1.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (sampleColor.equals(SampleColorDetected.BLUE)) {
            gamepad1.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (sampleColor.equals(SampleColorDetected.YELLOW)) {
            gamepad1.setLedColor(1, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepad1.setLedColor(0, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }

        double speedMultiplier = 0.35 + (1 - 0.35) * gamepad1.left_trigger;
        robot.follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * speedMultiplier, -gamepad1.left_stick_x * speedMultiplier, -gamepad1.right_stick_x * speedMultiplier, false);
        robot.follower.update();

        if (gamepad1.right_trigger > 0.01 &&
                !depositPivotState.equals(DepositPivotState.TRANSFER) &&
                robot.intake.getExtendoScaledPosition() <= (MAX_EXTENDO_EXTENSION - 5)) {
            robot.intake.target += 5;
        }

        super.run();

        String currentOperation = "Idle";
        String operationDetail = "None";
        if (intakePivotState == IntakePivotState.INTAKE && intakeMotorState == IntakeMotorState.FORWARD) {
            currentOperation = "Intaking";
            operationDetail = "Intake: " + intakePivotState + ", Motor: " + intakeMotorState;
        } else if (transferTriggered) {
            currentOperation = depositPivotState == DepositPivotState.SCORING ? "Scoring" : "Transferring";
            operationDetail = "Intake: " + intakePivotState + ", Deposit: " + depositPivotState + ", Transfer: " + (transferSuccessful ? "Successful" : "Pending/Failed");
        } else if (intakeMotorState == IntakeMotorState.REVERSE) {
            currentOperation = "Ejecting Sample";
            operationDetail = "Intake: " + intakePivotState + ", Motor: " + intakeMotorState;
        } else if (depositPivotState == DepositPivotState.SCORING) {
            currentOperation = "Manual Scoring";
            operationDetail = "Deposit: " + depositPivotState + ", Height: " + robot.deposit.target;
        }
        telemetryData.addData("Current Operation", currentOperation);
        telemetryData.addData("Operation Detail", operationDetail);

        telemetryData.addData("timer", timer.milliseconds());
        telemetryData.addData("autoEndPose", autoEndPose.toString());
        telemetryData.addData("extendoReached", robot.intake.extendoReached);
        telemetryData.addData("extendoRetracted", robot.intake.extendoRetracted);
        telemetryData.addData("slidesRetracted", robot.deposit.slidesRetracted);
        telemetryData.addData("slidesReached", robot.deposit.slidesReached);
        telemetryData.addData("opModeType", opModeType.name());
        telemetryData.addData("hasSample()", robot.intake.hasSample());
        telemetryData.addData("sampleDetected", sampleDetected);
        telemetryData.addData("transferTriggered", transferTriggered);
        telemetryData.addData("transferSuccessful", transferSuccessful);
        telemetryData.addData("colorSensor getDistance", robot.colorSensor.getDistance(DistanceUnit.CM));
        telemetryData.addData("Intake sampleColor", sampleColor);
        telemetryData.addData("correctSampleDetected", Intake.correctSampleDetected());
        telemetryData.addData("intakeMotorState", intakeMotorState);
        telemetryData.addData("liftTop.getPower()", robot.liftTop.getPower());
        telemetryData.addData("liftBottom.getPower()", robot.liftBottom.getPower());
        telemetryData.addData("extension.getPower()", robot.extension.getPower());
        telemetryData.addData("getExtendoScaledPosition()", robot.intake.getExtendoScaledPosition());
        telemetryData.addData("getLiftScaledPosition()", robot.deposit.getLiftScaledPosition());
        telemetryData.addData("slides target", robot.deposit.target);
        telemetryData.addData("extendo target", robot.intake.target);
        telemetryData.addData("intakePivotState", intakePivotState);
        telemetryData.addData("depositPivotState", depositPivotState);
        telemetryData.addData("Sigma", "Oscar");
        telemetryData.update();
        timer.reset();
        robot.ControlHub.clearBulkCache();
    }

    @Override
    public void end() {
        autoEndPose = robot.follower.getPose();
    }
}