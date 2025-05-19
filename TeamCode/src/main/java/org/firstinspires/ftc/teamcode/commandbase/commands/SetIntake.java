package org.firstinspires.ftc.teamcode.commandbase.commands;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commandbase.Intake;
import org.firstinspires.ftc.teamcode.hardware.Robot;

public class SetIntake extends CommandBase {
    private final Robot robot;

    private final Intake.IntakePivotState pivotState;

    private final double target;
    private final boolean clawIntakeOpen;


    ElapsedTime timer;
    private double previousServoPos;
    private double currentServoPos;


    private boolean waitForPivot = false;
    private boolean waitForSample = false;
    private boolean encoderReset = false;

    public SetIntake(Robot robot, Intake.IntakePivotState pivotState, double target, boolean clawOpen) {
        this.robot = robot;
        this.pivotState = pivotState;

        this.target = target;
        this.clawIntakeOpen =clawOpen;
        this.timer = new ElapsedTime();

        addRequirements(robot.intake);
    }


    @Override
    public void initialize() {
        // Update pivot and its variables for timing below
        previousServoPos = robot.leftIntakePivot.getPosition();
        robot.intake.setPivot(pivotState);
        currentServoPos = robot.leftIntakePivot.getPosition();
        timer.reset();

        // Update motor state and extendo
        // Wait for pivot if it is reversing the motor so that sample isn't shot into the robot


        robot.intake.setExtendoTarget(target);
        encoderReset = false; // Reset flag for encoder reset
    }

    @Override
    public void execute() {
//        if ((timer.milliseconds() > Math.abs(previousServoPos - currentServoPos) * INTAKE_PIVOT_MOVEMENT_TIME) && motorState.equals(Intake.IntakeMotorState.REVERSE) && waitForPivot) {
//            robot.intake.setActiveIntake(motorState);
//            waitForPivot = false;
//        }

        // Check if extendo has reached target and is retracted (assuming target = 0 is retracted)
        if (robot.intake.extendoReached && target == 0 && !encoderReset) {
            //robot.liftTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //robot.liftTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            encoderReset = true; // Prevent repeated resets
        }
    }

    // Command finishes when extendo has reached and pivot has had time to move
//    @Override
//    public boolean isFinished() {
//        if (waitForSample) {
//            switch (motorState) {
//                case FORWARD:
//                    return (robot.intake.extendoReached &&
//                            (timer.milliseconds() > Math.abs(previousServoPos - currentServoPos) * INTAKE_PIVOT_MOVEMENT_TIME))
//                            || (Intake.correctSampleDetected() && robot.intake.hasSample());
//                case REVERSE:
//                    return (robot.intake.extendoReached &&
//                            (timer.milliseconds() > Math.abs(previousServoPos - currentServoPos) * INTAKE_PIVOT_MOVEMENT_TIME + REVERSE_TIME_MS))
//                            || !robot.intake.hasSample();
//            }
//        }
//
//        return (robot.intake.extendoReached &&
//                (timer.milliseconds() > Math.abs(previousServoPos - currentServoPos) * INTAKE_PIVOT_MOVEMENT_TIME));
//    }
}