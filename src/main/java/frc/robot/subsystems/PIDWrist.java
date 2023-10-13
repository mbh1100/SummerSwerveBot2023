// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.testingdashboard.TestingDashboard;

public class PIDWrist extends PIDSubsystem {

  private static PIDWrist m_wrist;
  private CANSparkMax m_wristMotor;

  public static PIDWrist getInstance() {
    if (m_wrist == null) {
      m_wrist = new PIDWrist();
      TestingDashboard.getInstance().registerSubsystem(m_wrist, "PIDWrist");
    }
    return m_wrist;
  }

  /** Creates a new PIDWrist. */
  private PIDWrist() {
    super(
        // The PIDController used by the subsystem
        new PIDController(Constants.ManipulatorConstants.kWristP, 
                          Constants.ManipulatorConstants.kWristI, 
                          Constants.ManipulatorConstants.kWristD));
    m_wristMotor = new CANSparkMax(Constants.ManipulatorConstants.kWristMotorCanId, MotorType.kBrushless);
    m_wristMotor.getEncoder().setPositionConversionFactor(Constants.ManipulatorConstants.kWristDegreesPerPulse);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    m_wristMotor.set(output);
  }

  @Override
  public double getMeasurement() {
    return m_wristMotor.getEncoder().getPosition();
  }

  public void zeroEncoder()
  {
    m_wristMotor.getEncoder().setPosition(0);
  }

  public double getPosition()
  {
    return m_wristMotor.getEncoder().getPosition();
  }
}
