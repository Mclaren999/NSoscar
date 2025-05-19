package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commandbase.Deposit;
import org.firstinspires.ftc.teamcode.commandbase.Intake;
import org.firstinspires.ftc.teamcode.hardware.Robot;
@Deprecated
public class RealTransfer extends SequentialCommandGroup {
    public RealTransfer(Robot robot) {
        addCommands(
                new ParallelCommandGroup(
                        new SetDeposit(robot, Deposit.DepositPivotState.MIDDLE_HOLD, 0, true),
                        new SetIntake(robot, Intake.IntakePivotState.TRANSFER, 0, false)
                ),
                new ServoOnlyTransfer(robot)
        );
    }
}
