package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;

public class ExtendClimber extends CommandBase {
    private final Climber m_climber;

    public ExtendClimber(Climber climber) {
        m_climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        m_climber.setDesiredPose(270.0);
    }

    @Override
    public void execute() {
        m_climber.run();

    }

    @Override
    public void end(boolean interrupted) {
        m_climber.setDesiredPose(5.0);
    }
}
