package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

public class WaitForButtonCommand extends CommandBase {
    private final GamepadEx gamepad;
    private final GamepadKeys.Button button;

    public WaitForButtonCommand(GamepadEx gamepad, GamepadKeys.Button button) {
        this.gamepad = gamepad;
        this.button = button;
    }

    @Override
    public void initialize() {
        // No initialization needed
    }

    @Override
    public void execute() {
        // Continuously check for button press; no action needed in execute
    }

    @Override
    public boolean isFinished() {
        // Finish when the specified button is pressed
        return gamepad.getGamepadButton(button).get();
    }

    @Override
    public void end(boolean interrupted) {
        // No cleanup needed
    }
}