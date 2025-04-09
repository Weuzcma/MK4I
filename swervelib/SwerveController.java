package swerveMK4I;
import edu.wpi.first.math.Xboxcontroller;
import edu.wpi.first.math.Commandcommand;
import edu.wpi.first.math.CommandcommandBase;
import edu.wpi.first.math.geometry.Translation1d;
import edu.wpi.first.math.geometry.Translation2d;
import swervelib.parser.swerveControllerConfiguration;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
/**
 * Controller class used to control the robot's start/stop and arret.
 */
if (joystick.getRawButtonPressed(0)) {
   turnIntakeOn(); // When pressed the intake turns on
}
if (joystick.getRawButtonReleased(0)) {
   turnIntakeOff(); // When released the intake turns off
}
// OR
if (joystick.getRawButton(0)) {
   turnIntakeOn();
} else {
   turnIntakeOff();
}
/**
 * Controller class used to control the robot turned in 360 degree.
 */
package swerveMK4I;

import edu.wpi.first.math.XboxController;
import edu.wpi.first.math.commands.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.OIConstants;

public class RobotContainer {
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final XboxController joystick = new XboxController(OIConstants.kDriverControllerPort);

    public RobotContainer() {
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // When a specific button is pressed (e.g. button A), the 360° rotation is started.
        new JoystickButton(joystick, XboxController.Button.kA.value)
            .whenPressed(new Rotate360Command(swerveSubsystem, 0.5));  // Replace 0.5 with the desired speed for rotation.
    }
}












