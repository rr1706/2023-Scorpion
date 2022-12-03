package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ZeroClimber extends CommandBase {
    private final Climber m_climber;
    private final Timer m_timer = new Timer();
    private boolean m_finished = false;

    public ZeroClimber(Climber climber) {
        m_climber = climber;
        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
        m_climber.setPower(-0.20);
        m_finished = false;
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("Zeroing Climber", true);
        double time = m_timer.get();
        if (m_climber.getCurrent() >= 25.0 && time > 0.200){
            m_climber.setPoseRef(0.0);
            m_climber.stop();
            m_finished = true;
            m_climber.setDesiredPose(5.0);
        }

    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Zeroing Climber", false);
        m_climber.setDesiredPose(5.0);
        m_climber.stop();
        m_timer.stop();

    }

    @Override
    public boolean isFinished() {
        return m_finished;
    }
}
