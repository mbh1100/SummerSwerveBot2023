// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.testingdashboard.TestingDashboard;

public class Expel extends CommandBase {
  Intake m_intake;
  double m_speed;
  /** Creates a new Expel. */
  public Expel() {
    m_intake = Intake.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  public static void registerWithTestingDashboard() {
    Intake intake = Intake.getInstance();
    Expel cmd = new Expel();
    TestingDashboard.getInstance().registerCommand(intake, "Basic", cmd);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        m_speed = Constants.ManipulatorConstants.kIntakeExpelSpeed;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        m_intake.spin(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.spin(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
