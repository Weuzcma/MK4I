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
  this.m_DrivetrainSubSysstem = drivetraionSubsystem
  
  
