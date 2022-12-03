// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Utilities.JoystickLeftTrigger;
import frc.robot.Utilities.JoystickRightTrigger;
import frc.robot.commands.ClimbFromFloor;
import frc.robot.commands.DriveByController;
import frc.robot.commands.ExtendClimber;
import frc.robot.commands.IndexElevator;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SimpleShooter;
import frc.robot.commands.SmartFeed;
import frc.robot.commands.SmartShooter;
import frc.robot.commands.ZeroClimber;
import frc.robot.commands.ZeroHood;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Swerve.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import java.util.HashMap;

import com.pathplanner.lib.auto.BaseAutoBuilder;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  private final Drivetrain m_drive = new Drivetrain();
  private final Intake m_intake = new Intake(IntakeConstants.kLeftMotorID, IntakeConstants.kLeftAirPorts, "Only");
  private final Elevator m_elevator = new Elevator(ElevatorConstants.kLowMotorID, ElevatorConstants.kLowSensor, "Only");
  private final Shooter m_shooter = new Shooter(ShooterConstants.kMotorIDs);
  private final ShooterHood m_hood = new ShooterHood();
  private final Climber m_climber = new Climber();

  private final DriveByController m_driveByController 
    = new DriveByController(m_drive, m_driverController);
  private final IndexElevator m_Index = new IndexElevator(m_elevator);
  private final SmartShooter m_smartShooter = new SmartShooter(m_shooter, m_drive, m_hood, true, m_driverController);
  private final RunIntake m_runIntake = new RunIntake(m_intake);
  private final SmartFeed m_feed = new SmartFeed(m_elevator, m_drive, m_shooter, m_hood,m_operatorController);

  private final ExtendClimber m_extend = new ExtendClimber(m_climber);
  private final ClimbFromFloor m_climb = new ClimbFromFloor(m_climber);

  private final SimpleShooter m_defaultShoot = new SimpleShooter(m_shooter, 500);

  private final SimpleShooter m_testModeShoot = new SimpleShooter(m_shooter, 0.0);
  private final RunCommand m_stopIndex = new RunCommand(()->m_elevator.stop(), m_elevator);
  private final Command m_test = new ZeroHood(m_hood).alongWith(new ZeroClimber(m_climber)).alongWith(m_stopIndex).alongWith(m_testModeShoot);

  private final HashMap<String, Command> events = new HashMap<>();
  private final Command doNothin = new WaitCommand(20.0);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureAutoEvents();
    configureAutoChooser();

    m_drive.setDefaultCommand(m_driveByController);
    m_elevator.setDefaultCommand(m_Index);
    m_hood.setDefaultCommand(new RunCommand(() -> m_hood.run(1.0), m_hood));
    m_shooter.setDefaultCommand(m_defaultShoot);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new POVButton(m_driverController, 0)
        .onTrue(new InstantCommand(() -> m_drive.resetOdometry(new Rotation2d(0.0))));

    

    //new JoystickButton(m_driverController, Button.kY.value).whenPressed(()->m_drive.resetOdometry(new Pose2d(3.89,5.41, m_drive.getGyro().times(-1.0))));
    new JoystickButton(m_driverController, Button.kA.value).whileTrue(m_smartShooter);


    new JoystickRightTrigger(m_driverController).whileTrue(m_runIntake);
    new JoystickLeftTrigger(m_driverController).whileTrue(m_feed);
    //new JoystickButton(m_driverController, Button.kX.value).whileTrue(m_runIntake);
    //new JoystickButton(m_driverController, Button.kY.value).whileTrue(m_feed);
   // new JoystickButton(m_driverController, Button.kX.value).whenHeld(m_feed);

   new JoystickButton(m_operatorController, Button.kA.value).onTrue(m_extend).onFalse(m_climb);

  }

  private void configureAutoEvents () {
    events.put("startIntake", m_runIntake);
  }


  private void configureAutoChooser(){
    m_chooser.addOption("Do Nothing", doNothin);
    
    m_chooser.setDefaultOption("Do Nothing", doNothin);
    SmartDashboard.putData(m_chooser);  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
  }

  public Command getTest() {
    return m_test;
  }

}
