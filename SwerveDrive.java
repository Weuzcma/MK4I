package.drive.swerveMP4I;
import static edu.wpi.first.hal.FRCNetComm.tInstances.kRobotDriveSwerve_YAGSL;
import static edu.wpi.first.hal.FRCNetComm.tResourceType.kResourceType_RobotDrive;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Newtons;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import swervelib.encoders.CANCoderSwerve;
import swervelib.imu.Pigeon2Swerve;
import swervelib.imu.SwerveIMU;
import swervelib.math.SwerveMath;
import swervelib.motors.TalonFXSwerve;
import swervelib.parser.Cache;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.simulation.SwerveIMUSimulation;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
// Create a trajectory with the Trajectory class.

Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 0, new Rotation2d(0)), 
    List.of(new Translation2d(1, 1), new Translation2d(2, -1)), 
    new Pose2d(3, 0, new Rotation2d(Math.PI)), 
    new TrajectoryConfig(2.0, 2.0) // Vitesse max et accélération
);

// Use a RamseteController to follow this trajectory.
RamseteController ramseteController = new RamseteController();
SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    new SwerveModulePosition(...), new SwerveModulePosition(...)
);

// Implementation in robot loop for trajectory tracking.
Pose2d currentPose = swervePoseEstimator.getEstimatedPosition();
var ramseteOutput = ramseteController.calculate(currentPose, trajectory.sample(time));
ChassisSpeeds desiredSpeeds = ramseteOutput;
Pigeon2Swerve pigeon2 = new Pigeon2Swerve(0);
pigeon2.setYaw(0); // Reset robot orientation
Rotation2d currentAngle = pigeon2.getYaw();
Pose2d correctedPose = new Pose2d(currentPose.getTranslation(), currentAngle);
Field2d field = new Field2d();
field.setRobotPose(currentPose);
SmartDashboard.putData("Field", field);

// Telemetry to display robot speeds.
SmartDashboard.putNumber("Robot Speed X", chassisSpeeds.vxMetersPerSecond);
SmartDashboard.putNumber("Robot Speed Y", chassisSpeeds.vyMetersPerSecond);
SmartDashboard.putNumber("Robot Angular Speed", chassisSpeeds.omegaRadiansPerSecond);
double joystickX = driverController.getRawAxis(0);  // Joystick X
double joystickY = driverController.getRawAxis(1);  // Joystick Y
double joystickRot = driverController.getRawAxis(4);  // Joystick rotation
ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
    joystickX * maxSpeed, 
    joystickY * maxSpeed, 
    joystickRot * maxAngularSpeed
);
List<SwerveModuleState> states = swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
swerveDriveSubsystem.setModuleStates(states);
0
