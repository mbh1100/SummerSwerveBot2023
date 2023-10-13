// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.PIDWrist;

public class WristPreset2 extends CommandBase {
  PIDWrist m_wrist;
  /** Creates a new WristPreset1. */
  public WristPreset2() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_wrist = PIDWrist.getInstance();
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wrist.setSetpoint(Constants.ManipulatorConstants.kWristSetpoint90);
    m_wrist.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Do nothing here. The PID in the wrist does the work
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_wrist.getController().atSetpoint();
  }
}
