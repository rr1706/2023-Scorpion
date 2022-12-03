package frc.robot.commands;

import frc.robot.Constants.*;
import frc.robot.subsystems.Swerve.*;
import frc.robot.Utilities.FieldRelativeAccel;
import frc.robot.Utilities.FieldRelativeSpeed;
import frc.robot.Utilities.MathUtils;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

  /**
   * Implements a DriveByController command which extends the CommandBase class
   */
public class DriveByController extends CommandBase {
  private final Drivetrain m_robotDrive;
  private final XboxController m_controller;

  private boolean fieldOrient = true;

  /**
   * Contructs a DriveByController object which applys the driver inputs from the
   * controller to the swerve drivetrain
   * 
   * @param drive      is the swerve drivetrain object which should be created in
   *                   the RobotContainer class
   * @param controller is the user input controller object for controlling the
   *                   drivetrain
   */
  public DriveByController(Drivetrain drive, XboxController controller) {
    m_robotDrive = drive; // Set the private member to the input drivetrain
    m_controller = controller; // Set the private member to the input controller
    addRequirements(m_robotDrive); // Because this will be used as a default command, add the subsystem which will
                                   // use this as the default
  }

  @Override
  public void initialize(){
  }

  /**
   * the execute function is overloaded with the function to drive the swerve
   * drivetrain
   */
  @Override
  public void execute() {
    m_robotDrive.drive((
        -inputTransform(m_controller.getLeftY())
        * DriveConstants.kMaxSpeedMetersPerSecond),
        (
            -inputTransform(m_controller.getLeftX())
            * DriveConstants.kMaxSpeedMetersPerSecond),
        (-inputTransform(m_controller.getRightX())
            * DriveConstants.kMaxAngularSpeed),
        fieldOrient,true);
  }

  @Override
  public void end(boolean interrupted){

  }

  /**
   * This function takes the user input from the controller analog sticks, applys a deadband and then quadratically
   * transforms the input so that it is easier for the user to drive, this is especially important on high torque motors 
   * such as the NEOs or Falcons as it makes it more intuitive and easier to make small corrections
   * @param input is the input value from the controller axis, should be a value between -1.0 and 1.0
   * @return the transformed input value
   */
  private double inputTransform(double input){
    //return MathUtils.singedSquare(MathUtils.applyDeadband(input));
    return MathUtils.cubicLinear(MathUtils.applyDeadband(input), 0.9, 0.1);
  }

}
