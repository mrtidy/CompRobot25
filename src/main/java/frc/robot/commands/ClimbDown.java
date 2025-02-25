package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/**
 * This command is expected to be hooked to a trigger so it will execute when a
 * button is pressed and then be interrupted when the button is released.
 * 
 * The climber subsystem is used to move the robot down.
 */
public class ClimbDown extends Command {

    private final Climber climber;

    public ClimbDown(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.down();
    }
  
    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
  
    @Override
    public boolean isFinished() {
        return false;
    }
}
