// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import edu.wpi.first.math.MathUtil;

public class ArmOperatorRelativeAngleControl extends CommandBase {
  Arm m_arm;
  XboxController m_xbox;

  /** Creates a new ArmOperatorRelativeAngleControl. */
  public ArmOperatorRelativeAngleControl(XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = Arm.getInstance();
    m_xbox = controller;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.enableArmPid();
    m_arm.zeroEncoder();
    m_arm.setArmTargetAngle(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = m_arm.getArmTargetAngle();
    double input = MathUtil.applyDeadband(m_xbox.getLeftY(), Constants.ManipulatorConstants.kArmDeadband);
    angle += input*Constants.ManipulatorConstants.kArmIncrement;

    m_arm.setArmTargetAngle(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
