package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;

public class SimpleShooter extends CommandBase {
    private final Shooter m_shooter;
    private final double m_RPM;

    public SimpleShooter(Shooter shooter, double rpm) {
        m_shooter = shooter;
        m_RPM = rpm;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        m_shooter.run(m_RPM);
    }

    @Override
    public void execute() {
        m_shooter.run(m_RPM);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
    }

}
