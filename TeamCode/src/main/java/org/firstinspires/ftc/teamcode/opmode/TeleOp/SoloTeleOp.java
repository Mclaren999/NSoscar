package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.commandbase.Deposit.DepositPivotState;
import static org.firstinspires.ftc.teamcode.commandbase.Deposit.depositPivotState;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.IntakeMotorState;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.IntakePivotState;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.SampleColorDetected;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.SampleColorTarget;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.intakeMotorState;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.sampleColor;
import static org.firstinspires.ftc.teamcode.hardware.Globals.BACK_HIGH_SPECIMEN_HEIGHT;
import static org.firstinspires.ftc.teamcode.hardware.Globals.HIGH_BUCKET_HEIGHT;
import static org.firstinspires.ftc.teamcode.hardware.Globals.INTAKE_HOLD_SPEED;
import static org.firstinspires.ftc.teamcode.hardware.Globals.LOW_BUCKET_HEIGHT;
import static org.firstinspires.ftc.teamcode.hardware.Globals.MAX_EXTENDO_EXTENSION;
import static org.firstinspires.ftc.teamcode.hardware.Globals.OpModeType;
import static org.firstinspires.ftc.teamcode.hardware.Globals.autoEndPose;
import static org.firstinspires.ftc.teamcode.hardware.Globals.curPose;

import static org.firstinspires.ftc.teamcode.hardware.Globals.depositInit;
import static org.firstinspires.ftc.teamcode.hardware.Globals.opModeType;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.UninterruptibleCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commandbase.Deposit;
import org.firstinspires.ftc.teamcode.commandbase.Drive;
import org.firstinspires.ftc.teamcode.commandbase.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetAuto;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetDeposit;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetIntake;
import org.firstinspires.ftc.teamcode.commandbase.commands.UndoTransfer;
import org.firstinspires.ftc.teamcode.commandbase.commands.WaitForButtonCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.TelemetryData;

@TeleOp(name = "AAASoloTeleOp")
public class SoloTeleOp extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;

    public ElapsedTime timer;
    public ElapsedTime gameTimer;


    TelemetryData telemetryData = new TelemetryData(telemetry);

    private final Robot robot = Robot.getInstance();

    private boolean endgame = false;
    private boolean frontSpecimenScoring = false;
    private boolean isFirstDpadUpPress = true;
    private boolean isFirstDpadDownPress = true;
    private boolean isFirstTrianglePress = true;
    private boolean isFirstCirclePress = true;

    @Override
    public void initialize() {
        // Must have for all opModes
        opModeType = OpModeType.TELEOP;
        depositInit = DepositPivotState.MIDDLE_HOLD;

        INTAKE_HOLD_SPEED = 0;

        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        super.reset();

        robot.init(hardwareMap);

        // Initialize subsystems
        register(robot.deposit, robot.intake);

        robot.intake.setActiveIntake(IntakeMotorState.STOP);

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);




        // ========================================== OPERATOR ======================================

        operator.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> robot.deposit.setClawOpen(false)),
                        new WaitCommand(300),
                        new SetAuto(robot, DepositPivotState.BACK_SPECIMEN_SCORING, BACK_HIGH_SPECIMEN_HEIGHT, false).withTimeout(1500)
                )
        );

        operator.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                new UninterruptibleCommand(
//                        new InstantCommand(() -> robot.deposit.setClawOpen(true)),
                        new SetAuto(robot, DepositPivotState.FRONT_SPECIMEN_INTAKE, 0, true).withTimeout(1500)
                )
        );

        operator.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new UninterruptibleCommand(
                        new SequentialCommandGroup(
                                new SetDeposit(robot, Deposit.DepositPivotState.MIDDLE_HOLD, 0, true),
                                new SetIntake(robot, IntakePivotState.TRANSFER, IntakeMotorState.HOLD, 0, true),
                                new WaitCommand(200),
                                new SetDeposit(robot,DepositPivotState.TRANSFER,0,true),
                                new InstantCommand(() -> robot.deposit.setClawOpen(false)),
                                new WaitCommand(100),
                                new SetDeposit(robot, Deposit.DepositPivotState.MIDDLE_HOLD, 0, false).withTimeout(200)
                        )
                )
        );

        operator.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)),
                        new InstantCommand(() -> robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER))

                )
        );

        operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new SequentialCommandGroup(
                        new SetDeposit(robot, DepositPivotState.MIDDLE_HOLD, 0, false).withTimeout(1500),
                        new InstantCommand(() -> robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)),
                        new InstantCommand(() -> robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER))
                )

        );

        operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new ConditionalCommand(
                        new SetAuto(robot, DepositPivotState.SCORING, LOW_BUCKET_HEIGHT, false).withTimeout(1500),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.deposit.setClawOpen(true)),
                                new WaitCommand(300),
                                new SetDeposit(robot, DepositPivotState.MIDDLE_HOLD, 0, true).withTimeout(1500),
                                new InstantCommand(() -> robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)),
                                new InstantCommand(() -> robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER))
                        ),
                        () -> robot.deposit.target == 0
                )
        );

        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new UninterruptibleCommand(
                        new SequentialCommandGroup(
                                new ConditionalCommand(
                                        // First press: Move to SCORING with slides and claw closed
                                        new SequentialCommandGroup(
                                                new SetAuto(robot, DepositPivotState.PRESCORE, HIGH_BUCKET_HEIGHT, false).withTimeout(1500),
                                                new InstantCommand(() -> telemetryData.addData("DPAD UP Press", "First: Moving to SCORING"))
                                        ),
                                        // Second press: Adjust servos to SCORING, open claw, return to MIDDLE_HOLD
                                        new SequentialCommandGroup(
                                                new SetDeposit(robot, DepositPivotState.SCORING, HIGH_BUCKET_HEIGHT, false),
                                                new ParallelCommandGroup(
                                                        new WaitCommand( 175),
                                                        new InstantCommand(() -> robot.deposit.setClawOpen(true)),
                                                        new WaitCommand( 150)
                                                ),
                                                new SetDeposit(robot, DepositPivotState.MIDDLE_HOLD,  0,false).withTimeout(1500),
                                                new InstantCommand(() -> robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)),
                                                new InstantCommand(() -> robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER))
                                        ),
                                        ()-> isFirstDpadUpPress
                                ),
                                // Toggle the press state
                                new InstantCommand(() -> isFirstDpadUpPress = !isFirstDpadUpPress)
                        )
                )
        );

        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new UninterruptibleCommand(
                        new SequentialCommandGroup(
                                new ConditionalCommand(
                                        // First press: Move to PRESCORE with slides and claw closed
                                        new SequentialCommandGroup(
                                                new SetAuto(robot, DepositPivotState.PRESCORE, HIGH_BUCKET_HEIGHT, false).withTimeout(1500),
                                                new InstantCommand(() -> telemetryData.addData("DPAD_DOWN Press", "First: Moving to PRESCORE"))
                                        ),
                                        // Second press: Adjust servos to SCORING, wait for right bumper, open claw, return to MIDDLE_HOLD
                                        new SequentialCommandGroup(
                                                new SetAuto(robot, DepositPivotState.SCORING, HIGH_BUCKET_HEIGHT, false).withTimeout(1500),
                                                new WaitCommand(400),
                                                new WaitForButtonCommand(operator, GamepadKeys.Button.DPAD_DOWN),
                                                new InstantCommand(() -> robot.deposit.setClawOpen(true)),
                                                new WaitCommand(500),
                                                new SetDeposit(robot, DepositPivotState.MIDDLE_HOLD, 0, true).withTimeout(1500),
                                                new InstantCommand(() -> robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)),
                                                new InstantCommand(() -> robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)),
                                                new InstantCommand(() -> telemetryData.addData("DPAD_DOWN Press", "Second: Completed cycle"))
                                        ),
                                        () -> isFirstDpadDownPress
                                ),
                                // Toggle the press state
                                new InstantCommand(() -> isFirstDpadDownPress = !isFirstDpadDownPress)
                        )
                )
        );

        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> robot.deposit.setClawOpen(false))
        );

        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(

                new InstantCommand(() -> robot.deposit.setClawOpen(true))
        );

        operator.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
                new UninterruptibleCommand(
                        new SetDeposit(robot, DepositPivotState.SCORING, HIGH_BUCKET_HEIGHT, false).withTimeout(1500)
                )
        );

        operator.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(
                new UninterruptibleCommand(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> robot.deposit.setClawOpen(true)),
                                        new WaitCommand(300),
                                        new SetDeposit(robot, DepositPivotState.MIDDLE_HOLD, 0, true).withTimeout(1500)
                                ),
                                new InstantCommand(),
                                () -> robot.deposit.target == HIGH_BUCKET_HEIGHT
                        )
                )
        );

        operator.getGamepadButton(GamepadKeys.Button.PS).whenPressed(
                new InstantCommand()
        );

        operator.getGamepadButton(GamepadKeys.Button.OPTIONS).whenPressed(
                new InstantCommand()
        );

        operator.getGamepadButton(GamepadKeys.Button.START).whenPressed(
                new InstantCommand()
        );

        operator.getGamepadButton(GamepadKeys.Button.TOUCHPAD).whenPressed(
                new InstantCommand()
        );








        // ========================================== DRIVER ======================================


        driver.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new SequentialCommandGroup(
                        new SetIntake(robot, IntakePivotState.INTAKE_READY, intakeMotorState, MAX_EXTENDO_EXTENSION, true),
                        new InstantCommand(() -> robot.intake.toggleActiveIntake(SampleColorTarget.ANY_COLOR))
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
            new UninterruptibleCommand(
                    new SequentialCommandGroup(
                            new ConditionalCommand(
                                    // First press: Move to PRESCORE with slides and claw closed
                                    new SequentialCommandGroup(
                                            new SetIntake(robot, IntakePivotState.INTAKE_READY, IntakeMotorState.FORWARD, MAX_EXTENDO_EXTENSION, true)
                                    ),
                                    // Second press: Adjust servos to SCORING, wait for right bumper, open claw, return to MIDDLE_HOLD
                                    new SequentialCommandGroup(
                                            new SetIntake(robot, IntakePivotState.INTAKE, IntakeMotorState.FORWARD, MAX_EXTENDO_EXTENSION, true)
                                    ),
                                    () -> isFirstCirclePress
                            ),
                            // Toggle the press state
                            new InstantCommand(() -> isFirstCirclePress = !isFirstCirclePress)
                    )
            )
        );

        driver.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                new SequentialCommandGroup(
                        new SetIntake(robot, IntakePivotState.INTAKE_READY, IntakeMotorState.FORWARD, 0, true),
                        new WaitCommand(800)
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(

                new UninterruptibleCommand(
                        new SequentialCommandGroup(
                                new ConditionalCommand(
                                        // First press
                                        new SequentialCommandGroup(
                                                new ParallelCommandGroup(
                                                        new InstantCommand(() -> robot.drive.setSubPusher(Drive.SubPusherState.OUT)),
                                                        new SetIntake(robot, IntakePivotState.INTAKE, IntakeMotorState.FORWARD, 0, true)
                                                )
                                        ),
                                        // Second press
                                        new ParallelCommandGroup(
                                                new SetIntake(robot, IntakePivotState.INTAKE, IntakeMotorState.FORWARD, MAX_EXTENDO_EXTENSION, true),
                                                new SequentialCommandGroup(
                                                        new WaitCommand(200),
                                                        new InstantCommand(() -> robot.drive.setSubPusher(Drive.SubPusherState.IN))
                                                )
                                        ),
                                        () -> isFirstTrianglePress
                                ),
                                // Toggle the press state
                                new InstantCommand(() -> isFirstTrianglePress = !isFirstTrianglePress)
                        )
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand(() -> robot.intake.setExtendoTarget(0))
        );

        // TO-DO: need to make into 1 method in Drive.java
        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new ConditionalCommand(
                        new InstantCommand(() -> robot.drive.setSubPusher(Drive.SubPusherState.OUT)),
                        new InstantCommand(() -> robot.drive.setSubPusher(Drive.SubPusherState.IN)),
                        () -> Drive.subPusherState.equals(Drive.SubPusherState.IN))
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new SequentialCommandGroup(
                    new InstantCommand(() -> telemetryData.addData("Degrees", robot.getYawDegrees())),
                    new InstantCommand(() -> robot.follower.setTeleOpMovementVectors(0,0, robot.getYawDegrees() , false))
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new UninterruptibleCommand(
                        new RepeatCommand(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> robot.follower.setStartingPose(new Pose(0, 0, Math.toRadians(0)))),
                                        new FollowPathCommand(robot.follower,
                                                robot.follower.pathBuilder()
                                                        .addPath(
                                                                new BezierLine(
                                                                        new Point(robot.follower.getPose().getX(), robot.follower.getPose().getY(), Point.CARTESIAN),
                                                                        new Point(0, 0, Point.CARTESIAN)
                                                                )
                                                        )
                                                        .setLinearHeadingInterpolation(Math.toRadians(robot.follower.getTotalHeading()), Math.toRadians(180))
                                                        .build(),
                                                true
                                        )
                                ))
                ).andThen(
                        new InstantCommand(() -> robot.follower.startTeleopDrive())
                )
        );


        driver.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
                new UninterruptibleCommand(
                        new SequentialCommandGroup(
                                new UndoTransfer(robot),
                                new SetIntake(robot, IntakePivotState.INTAKE, IntakeMotorState.REVERSE, MAX_EXTENDO_EXTENSION, true)
                        )
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(
                new UninterruptibleCommand(
                        new SequentialCommandGroup(
                                new UndoTransfer(robot),
                                new SetIntake(robot, IntakePivotState.INTAKE, IntakeMotorState.REVERSE, MAX_EXTENDO_EXTENSION, true)
                        )
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.PS).whenPressed(
                new UninterruptibleCommand(
                        new RepeatCommand(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> curPose = robot.follower.getPose()),
                                        new FollowPathCommand(robot.follower,
                                                robot.follower.pathBuilder()
                                                        .addPath(
                                                                new BezierLine(
                                                                        new Point(robot.follower.getPose().getX(), robot.follower.getPose().getY(), Point.CARTESIAN),
                                                                        new Point(15, 115.1, Point.CARTESIAN)
                                                                )
                                                        )
                                                        .setLinearHeadingInterpolation(Math.toRadians(369), Math.toRadians(-45))
                                                        .build(),
                                                true
                                        )
                                ))
                ).andThen(
                        new InstantCommand(() -> robot.follower.startTeleopDrive())
                )
        );

        operator.getGamepadButton(GamepadKeys.Button.OPTIONS).whenPressed(
                new InstantCommand()
        );

        operator.getGamepadButton(GamepadKeys.Button.START).whenPressed(
                new InstantCommand()
        );

        operator.getGamepadButton(GamepadKeys.Button.TOUCHPAD).whenPressed(
                new InstantCommand()
        );


        super.run();
    }

    @Override
    public void run() {
        // Keep all the has movement init for until when TeleOp starts
        // This is like the init but when the program is actually started
        new UninterruptibleCommand(
                new RepeatCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.follower.setStartingPose(new Pose(6.25, 30, Math.toRadians(180)))),
                                new FollowPathCommand(robot.follower,
                                        robot.follower.pathBuilder()
                                                .addPath(
                                                        new BezierCurve(
                                                                new Point(robot.follower.getPose().getX(), robot.follower.getPose().getY(), Point.CARTESIAN),
                                                                new Point(6.25, 30, Point.CARTESIAN),
                                                                new Point(63.706, 117.899, Point.CARTESIAN),
                                                                new Point(62.157, 100, Point.CARTESIAN)
                                                        )
                                                )
                                                .setConstantHeadingInterpolation(Math.toRadians(0)).build(),
                                        true
                                )
                        ))
        ).andThen(
                new InstantCommand(() -> robot.follower.startTeleopDrive())

        );
        super.run();
        if (timer == null) {
            robot.initHasMovement();

            INTAKE_HOLD_SPEED = 0.15; // Enable hold

            timer = new ElapsedTime();
            gameTimer = new ElapsedTime();
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

        // purple is back (default) spec scoring, green is front spec scoring
        if (frontSpecimenScoring) {
            gamepad2.setLedColor(0, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepad2.setLedColor(1, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        }

        // Pinpoint Field Centric Code
        double speedMultiplier = 0.35 + (1 - 0.35) * gamepad1.left_trigger;
        robot.follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * speedMultiplier, -gamepad1.left_stick_x * speedMultiplier , -gamepad1.right_stick_x * speedMultiplier* 0.6 , false);
        robot.follower.update();

        // Manual control of extendo
        if (gamepad1.right_trigger > 0.01 &&
            !depositPivotState.equals(DepositPivotState.TRANSFER) &&
            robot.intake.getExtendoScaledPosition() <= (MAX_EXTENDO_EXTENSION - 5)) {

            robot.intake.target += 5;
        }

        // Hang
        if (gamepad2.left_trigger > 0.5) {
            robot.drive.setHang(Drive.HangState.STOP);
        }

        // DO NOT REMOVE! Runs FTCLib Command Scheduler
        super.run();

        telemetryData.addData("timer", timer.milliseconds());
//        telemetryData.addData("autoEndPose", autoEndPose.toString());
//        telemetryData.addData("extendoReached", robot.intake.extendoReached);
//        telemetryData.addData("extendoRetracted", robot.intake.extendoRetracted);
//        telemetryData.addData("slidesRetracted", robot.deposit.slidesRetracted);
//        telemetryData.addData("slidesReached", robot.deposit.slidesReached);
        telemetryData.addData("opModeType", opModeType.name());
//
//        telemetryData.addData("hasSample()", robot.intake.hasSample());
        telemetryData.addData("colorSensor getDistance", robot.colorSensor.getDistance(DistanceUnit.CM));
        telemetryData.addData("Intake sampleColor", sampleColor);
//        telemetryData.addData("correctSampleDetected", Intake.correctSampleDetected());
//        telemetryData.addData("intakeMotorState", intakeMotorState);
//
//        telemetryData.addData("liftTop.getPower()", robot.liftTop.getPower());
//        telemetryData.addData("liftBottom.getPower()", robot.liftBottom.getPower());
//        telemetryData.addData("extension.getPower()", robot.extension.getPower());
//
        telemetryData.addData("getExtendoScaledPosition()", robot.intake.getExtendoScaledPosition());
        telemetryData.addData("getLiftScaledPosition()", robot.rightFront.getPosition());
//
        telemetryData.addData("slides target", robot.deposit.target);
        telemetryData.addData("extendo target", robot.intake.target);
//
//        telemetryData.addData("intakePivotState", intakePivotState);
//        telemetryData.addData("depositPivotState", depositPivotState);
        telemetryData.addData("Sigma", "NORTHSTAR");

        telemetryData.update(); // DO NOT REMOVE! Needed for telemetry
        timer.reset();
        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        // Also only clearing the control hub to decrease loop times
        // This means if we start reading both hubs (which we aren't) we need to clear both
        robot.ControlHub.clearBulkCache();
    }

    @Override
    public void end() {
        autoEndPose = robot.follower.getPose();
    }
}