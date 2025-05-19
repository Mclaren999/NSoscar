package org.firstinspires.ftc.teamcode.opmode.Auto;

import static org.firstinspires.ftc.teamcode.commandbase.Deposit.*;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.*;
import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commandbase.Deposit;
import org.firstinspires.ftc.teamcode.commandbase.Drive;
import org.firstinspires.ftc.teamcode.commandbase.Intake;
import org.firstinspires.ftc.teamcode.commandbase.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commandbase.commands.HoldPointCommand;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetAuto;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetDeposit;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetIntake;
import org.firstinspires.ftc.teamcode.commandbase.commands.UndoTransfer;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.TelemetryData;

import java.util.ArrayList;

@Config
@Autonomous(name = "Burrito Bowl2 (0spec+4sample)", group = "Chipotle Menu", preselectTeleOp = "FullTeleOp")
public class BurritoBowl2 extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private ElapsedTime timer;
    private final ArrayList<PathChain> paths = new ArrayList<>();
    TelemetryData telemetryData = new TelemetryData(telemetry);
    private DashboardPoseTracker dashboardPoseTracker;

    public void generatePath() {
        robot.follower.setStartingPose(new Pose(0, 105.125, Math.toRadians(270)));

        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(0, 105.13, Point.CARTESIAN),
                                        new Point(0, 111, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(-35))
                        .setReversed(true)
                        .build());
        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(0, 111, Point.CARTESIAN),
                                        new Point(20, 121.945, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                        .build());
        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(20, 121.945, Point.CARTESIAN),
                                        new Point(17, 128.1, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(369), Math.toRadians(-40))
                        .build());
        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(17, 128.1, Point.CARTESIAN),
                                        new Point(24, 132.3, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(0))
                        .build());
        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(24, 132.3, Point.CARTESIAN),
                                        new Point(18, 130.1, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(369), Math.toRadians(-45))
                        .build());
        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(18, 130.1, Point.CARTESIAN),
                                        new Point(22.7, 139.1, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(18))
                        .build());
        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(22.7, 139.1, Point.CARTESIAN),
                                        new Point(20, 132.1, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(-35))
                        .build());
        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(20, 132.1, Point.CARTESIAN),
                                        new Point(63.706, 117.899, Point.CARTESIAN),
                                        new Point(60.157, 90, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-90))
                        .build());
//        robot.follower.pathBuilder()
//                .addPath(
//                        // Line 8
//                        new BezierCurve(
//                                new Point(60.157, 90, Point.CARTESIAN),
//                                new Point(63.706, 117.899, Point.CARTESIAN),
//                                new Point(20, 133.1, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(-45))
//                .build());
    }

    private SequentialCommandGroup intakeSampleCycleHalf(int pathNum, int extendoTarget) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        //new SetDeposit(robot, DepositPivotState.MIDDLE_HOLD, 0, true).withTimeout(1000),
                        new FollowPathCommand(robot.follower, paths.get(pathNum)).setHoldEnd(true),
                        new SequentialCommandGroup(
                                new SetIntake(robot, IntakePivotState.INTAKE, IntakeMotorState.FORWARD, 120, true),
//                                new WaitCommand(400),
                                new SetIntake(robot, IntakePivotState.INTAKE, IntakeMotorState.FORWARD, extendoTarget, true),
                                new ParallelRaceGroup(
                                        new WaitUntilCommand(robot.intake::hasSample)
                                ).withTimeout(1500)
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(100),
                                new SetDeposit(robot, DepositPivotState.MIDDLE_HOLD, 0, true).withTimeout(600)
                        )
                ),
                new WaitCommand(150),
                new InstantCommand(() -> robot.intake.setActiveIntake(IntakeMotorState.HOLD)),
                new WaitCommand(150),
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, paths.get(pathNum)).setHoldEnd(true),
                        new SequentialCommandGroup(
                                new SetIntake(robot, IntakePivotState.TRANSFER, IntakeMotorState.HOLD, 0, true),
                                new WaitCommand(100),
                                new SetDeposit(robot, DepositPivotState.TRANSFER, 0, true).withTimeout(600),
                                new WaitCommand(350),
                                new InstantCommand(() -> robot.deposit.setClawOpen(false))
                        )
                )
        );
    }

    private SequentialCommandGroup intakeSubSampleCycleHalf(int pathNum) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        //new SetDeposit(robot, DepositPivotState.MIDDLE_HOLD, 0, true).withTimeout(1000),
                        new FollowPathCommand(robot.follower, paths.get(pathNum)).setHoldEnd(true),
                        new SequentialCommandGroup(
                                new WaitCommand(100),
                                new SetDeposit(robot, Deposit.DepositPivotState.MIDDLE_HOLD, 0, true).withTimeout(800)
                        )
                ),
                new InstantCommand(() -> robot.drive.setSubPusher(Drive.SubPusherState.OUT)),
                new WaitCommand(250),
                new InstantCommand(() -> robot.drive.setSubPusher(Drive.SubPusherState.IN)),
                new WaitCommand(250),
                new ParallelRaceGroup(
                        new WaitUntilCommand(() -> Intake.correctSampleDetected() && robot.intake.hasSample()),
                        new SequentialCommandGroup(
                                new RepeatCommand(
                                        new SequentialCommandGroup(
                                                new SetIntake(robot, Intake.IntakePivotState.INTAKE, IntakeMotorState.FORWARD, 0, true),
                                                new WaitCommand(150),
                                                new SetIntake(robot, Intake.IntakePivotState.INTAKE, IntakeMotorState.FORWARD, MAX_EXTENDO_EXTENSION, true),
                                                new WaitUntilCommand(() -> robot.intake.hasSample() && !Intake.correctSampleDetected()).withTimeout(1500),
                                                new ConditionalCommand(
                                                        new HoldPointCommand(robot.follower, new Pose(-3, 0, 0), false), // Right 3 inches
                                                        new HoldPointCommand(robot.follower, new Pose(3, 0, 0), false), // Left 3 inches
                                                        () -> robot.follower.getPose().getX() > 60
                                                ).withTimeout(500)
                                        )
                                )
                        )
                ).withTimeout(4000),
                new InstantCommand(() -> robot.intake.setActiveIntake(IntakeMotorState.HOLD)),
                new WaitCommand(150),
                new SetIntake(robot, IntakePivotState.TRANSFER, IntakeMotorState.HOLD, 0, true),
                new WaitCommand(100),
                new SetDeposit(robot, DepositPivotState.TRANSFER, 0, true).withTimeout(600),
                new WaitCommand(350),
                new InstantCommand(() -> robot.deposit.setClawOpen(false)),
                new WaitCommand(100)
        );
    }

    private ParallelCommandGroup scoreSampleCycleHalf(int pathNum) {
        return new ParallelCommandGroup(
            new FollowPathCommand(robot.follower, paths.get(pathNum)).setHoldEnd(true),
            new SequentialCommandGroup(
                    new WaitCommand(200),
                    new SetAuto(robot, DepositPivotState.PRESCORE, HIGH_BUCKET_HEIGHT, false).withTimeout(1000),
                    new SetAuto(robot, DepositPivotState.SCORING, HIGH_BUCKET_HEIGHT, false).withTimeout(800),
                    new InstantCommand(() -> robot.deposit.setClawOpen(true)),
                    new WaitCommand(400)
            )
        );
    }

    private Command scoreSampleCycleHalfLast(int pathNum) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, paths.get(pathNum)).setHoldEnd(true),
                        new SequentialCommandGroup(
                                new WaitCommand(400),
                                new SetAuto(robot, DepositPivotState.PRESCORE, HIGH_BUCKET_HEIGHT, false).withTimeout(1000),
                                new SetAuto(robot, DepositPivotState.SCORING, HIGH_BUCKET_HEIGHT, false).withTimeout(800),
                                new InstantCommand(() -> robot.deposit.setClawOpen(true)),
                                new WaitCommand(400)
                        )
                ),
                new SetDeposit(robot, DepositPivotState.MIDDLE_HOLD, 0, true).withTimeout(1000)
        );
    }

    @Override
    public void initialize() {
        opModeType = OpModeType.AUTO;
        depositInit = DepositPivotState.MIDDLE_HOLD;
        Intake.sampleColorTarget = SampleColorTarget.ANY_COLOR;

        timer = new ElapsedTime();
        timer.reset();

        super.reset();
        robot.init(hardwareMap);
        register(robot.deposit, robot.intake);
        robot.initHasMovement();
        robot.follower.setMaxPower(0.9);
        FollowerConstants.zeroPowerAccelerationMultiplier = 11;

        generatePath();

        schedule(
                new RunCommand(() -> robot.follower.update()),
                new SequentialCommandGroup(
                        // Sample 1
                        new ParallelCommandGroup(
                                new SetAuto(robot, DepositPivotState.PRESCORE, HIGH_BUCKET_HEIGHT, false).withTimeout(1000),
                                new WaitCommand(200),

                                new FollowPathCommand(robot.follower, paths.get(0)).setHoldEnd(true)
                        ),
                        new WaitCommand(200),
                        new SetAuto(robot, DepositPivotState.SCORING, HIGH_BUCKET_HEIGHT, false).withTimeout(1000),
                        new InstantCommand(() -> robot.deposit.setClawOpen(true)),
                        new WaitCommand(400),
                        new SetDeposit(robot, DepositPivotState.MIDDLE_HOLD, 0, true).withTimeout(1000),

                        // Sample 2
                        intakeSampleCycleHalf(1, 370),
                        scoreSampleCycleHalf(2),

                        // Sample 3
                        intakeSampleCycleHalf(3, 340),
                        scoreSampleCycleHalf(4),

                        // Sample 4
                        intakeSampleCycleHalf(5, 330),
                        scoreSampleCycleHalf(6),

                        // Park
                        new InstantCommand(() -> robot.follower.setMaxPower(0.9)),
                        new InstantCommand(() -> FollowerConstants.zeroPowerAccelerationMultiplier = 11),
                        intakeSubSampleCycleHalf(7),
                        scoreSampleCycleHalfLast(8)
                )
        );

        dashboardPoseTracker = new DashboardPoseTracker(robot.poseUpdater);
        Drawing.drawRobot(robot.poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }

    @Override
    public void run() {
        super.run();

        telemetryData.addData("timer", timer.milliseconds());
        telemetryData.addData("extendoReached", robot.intake.extendoReached);
        telemetryData.addData("slidesRetracted", robot.deposit.slidesRetracted);
        telemetryData.addData("slidesReached", robot.deposit.slidesReached);
        telemetryData.addData("robotState", Robot.robotState);
        telemetryData.addData("hasSample()", robot.intake.hasSample());
        telemetryData.addData("colorSensor getDistance", robot.colorSensor.getDistance(DistanceUnit.CM));
        telemetryData.addData("intakePivotState", intakePivotState);
        telemetryData.addData("depositPivotState", depositPivotState);
        telemetryData.addData("UndoTransfer", CommandScheduler.getInstance().isScheduled(new UndoTransfer(robot)));
        telemetryData.addData("liftTop.getPower()", robot.liftTop.getPower());
        telemetryData.addData("liftBottom.getPower()", robot.liftBottom.getPower());
        telemetryData.addData("deposit target", robot.deposit.target);
        telemetryData.addData("liftEncoder.getPosition()", robot.liftEncoder.getPosition());
        telemetryData.addData("extendo target", robot.intake.target);
        telemetryData.addData("extensionEncoder.getPosition()", robot.extensionEncoder.getPosition());
        telemetryData.update();

        dashboardPoseTracker.update();
        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(robot.poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();

        robot.ControlHub.clearBulkCache();
    }

    @Override
    public void end() {
        autoEndPose = robot.follower.getPose();
    }
}