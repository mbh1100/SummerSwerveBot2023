// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.subsystems.PIDWrist;

public class WristManualControl extends CommandBase {

  PIDWrist m_wrist;
  XboxController m_xbox;

  /** Creates a new WristManualControl. */
  public WristManualControl(XboxController controller) {
      // Use addRequirements() here to declare subsystem dependencies.
    m_wrist = PIDWrist.getInstance();
    m_xbox = controller;
    addRequirements(m_wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wrist.zeroEncoder();
    m_wrist.setSetpoint(0);
    m_wrist.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = m_wrist.getPosition();
    double input = MathUtil.applyDeadband(m_xbox.getRightX(), Constants.ManipulatorConstants.kWristDeadband); 
    angle += input * Constants.ManipulatorConstants.kWristIncrement;
    m_wrist.setSetpoint(angle);
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
