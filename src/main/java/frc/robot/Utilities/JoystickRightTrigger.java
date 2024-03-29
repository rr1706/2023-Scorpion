package frc.robot.Utilities;

import edu.wpi.first.wpilibj.XboxController;
import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;
import edu.wpi.first.wpilibj2.command.button.*;

public class JoystickRightTrigger extends Trigger {

  /**
   * Create a button for triggering commands off a controller's analog axis
   * 
   * @param controller The controller to use
   * @param side Which side of the controller (Left = true, Right = false)
   */
  public JoystickRightTrigger(XboxController controller) {
    this(controller, 0.25);
  }

  /**
   * Create a button for triggering commands off a controller's analog axis
   * 
   * @param controller The controller to use
   * @param side Which side of the controller (Left = true, Right = false)
   */
  public JoystickRightTrigger(XboxController controller, double threshold) {
    super(() -> controller.getRightTriggerAxis() >= threshold);
    requireNonNullParam(controller, "joystick", "JoystickButton");
  }


}