package frc.robot.commands;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.commands.CommandBase;
import frc.robot.subsytems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
  private final DrivetraionSubsyetem m_drivetrainSubsystem;

private final DoubleSupplier m_translationXsupplier;
private final DoubleSupllier m_translationYsupplier;
private final DoubleSupplier m_rotationSupplier;

public DefaultDrirveCommand(DrivetrainSubsystem drivetrainSubsystem dricvetrainSubsystem,
                            DoubleSupplier translationXsupplier,
                            DoubleSupplier translationYsupplier,
                            DoubleSupplier rotationSupplier) {
  this.m_DrivetrainSubSysstem = drivetrainSubsystem;
  this.m_DrivetrainSubsystem = translationXsupplier;
  this.m_translationYsupplier = translationYsupplier;
  this.m_rotationSupplier = rotationSupplier;
  addRequirements(dirvetrainSubsystem);
}

  @override
  public void execute() {
// you can use 'neww ChassisSpeeds(...)' for robot-orientation movement instead of field-orientation movement
    m_drivetrainSubsystem.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(
              m_translationXsupplier.getAsDouble(),
              m_translationYsupplier.getAsDouble(),
              m_rotationSupllier.getAsDouble(),
        )
      );
  }

  @Override
  public void  end(boolen interrupted) {
    m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }
}
  
  
