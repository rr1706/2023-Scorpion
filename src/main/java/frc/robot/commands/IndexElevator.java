package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class IndexElevator extends CommandBase {

    private final Elevator m_top;
    private final XboxController m_operator;

    public IndexElevator(Elevator top,  XboxController operator) {
        m_top = top;
        m_operator = operator;
    }

    public IndexElevator(Elevator top) {
        m_top = top;
        addRequirements(top);
        m_operator = new XboxController(3);
    }

    @Override
    public void execute() {

        if (m_top.getSensor()) {
            m_top.stop();
            //SmartDashboard.putBoolean("top index", true);

        } else{
            m_top.run(0.75);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_top.stop();
    }
}
