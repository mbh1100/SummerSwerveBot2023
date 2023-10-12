// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.testingdashboard.TestingDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Arm extends SubsystemBase {
  private static Arm m_arm;
  private CANSparkMax m_armRightMotor;
  private CANSparkMax m_armLeftMotor;
  private RelativeEncoder m_armEncoder;
  private PIDController m_armPid;
  private double m_armTargetAngle;
  private boolean m_armPidEnable;

  /** Creates a new Arm. */
  private Arm() {
    m_armRightMotor = new CANSparkMax(Constants.ManipulatorConstants.kArmMotorRightCanId, MotorType.kBrushless);
    m_armLeftMotor = new CANSparkMax(Constants.ManipulatorConstants.kArmMotorLeftCanId, MotorType.kBrushless);
    m_armRightMotor.restoreFactoryDefaults();
    m_armLeftMotor.restoreFactoryDefaults();
    m_armRightMotor.setSmartCurrentLimit(Constants.ManipulatorConstants.kArmCurrentLimit);
    m_armLeftMotor.setSmartCurrentLimit(Constants.ManipulatorConstants.kArmCurrentLimit);

    m_armEncoder = m_armRightMotor.getEncoder();

    m_armRightMotor.setIdleMode(IdleMode.kBrake);
    m_armLeftMotor.setIdleMode(IdleMode.kBrake);

    m_armLeftMotor.follow(m_armRightMotor, true);
    zeroEncoder();

    m_armPid = new PIDController(Constants.ManipulatorConstants.kArmP, Constants.ManipulatorConstants.kArmI, Constants.ManipulatorConstants.kArmD);

    m_armTargetAngle = 0;
  }

  public static Arm getInstance() {
    if (m_arm == null) {
      m_arm = new Arm();
      TestingDashboard.getInstance().registerSubsystem(m_arm, "Arm");
      TestingDashboard.getInstance().registerNumber(m_arm, "Encoders", "ArmEncoder", 0);
      TestingDashboard.getInstance().registerNumber(m_arm, "Encoders", "ArmEncoderPulses", 0);
      TestingDashboard.getInstance().registerNumber(m_arm, "MotorCurrents", "ArmCurrentR", 0);
      TestingDashboard.getInstance().registerNumber(m_arm, "MotorCurrents", "ArmCurrentL", 0);
      TestingDashboard.getInstance().registerString(m_arm, "PidControl", "ArmSoftwarePidEnable", "Disabled");
      TestingDashboard.getInstance().registerNumber(m_arm, "ArmSoftwarePID", "TargetArmAngle", 0);
      TestingDashboard.getInstance().registerNumber(m_arm, "ArmSoftwarePID", "TargetArmTolerance", Constants.ManipulatorConstants.kArmPidTolerance);
      TestingDashboard.getInstance().registerNumber(m_arm, "ArmSoftwarePID", "TargetArmP", Constants.ManipulatorConstants.kArmP);
      TestingDashboard.getInstance().registerNumber(m_arm, "ArmSoftwarePID", "TargetArmI", Constants.ManipulatorConstants.kArmI);
      TestingDashboard.getInstance().registerNumber(m_arm, "ArmSoftwarePID", "TargetArmD", Constants.ManipulatorConstants.kArmD);

      TestingDashboard.getInstance().registerNumber(m_arm, "ArmAngles", "ArmAngle", 0);
    }
    return m_arm;
  }

  public void zeroEncoder() {
    m_armEncoder.setPosition(0);
  }

  public RelativeEncoder getEncoder() {
    return m_armEncoder;
  }

  private double getArmAngle() {
    return m_armEncoder.getPosition() * Constants.ManipulatorConstants.kArmDegreesPerPulse;
  }

  // Set the target Arm angle
  public void setArmTargetDegrees(int degrees)
  {
    setArmTargetAngle(degrees);
  }

  public void setArmTargetAngle(double angle) {
    m_armTargetAngle = angle;
  }

  public double getArmTargetAngle() {
    return m_armTargetAngle;
  }

  public PIDController getArmPID() {
    return m_armPid;
  }

  //* turn on the PID */
  public void enableArmPid() {
    m_armPidEnable = true;
    m_armPid.reset();
  }

  // Turn off the Arm PID controller
  public void disableArmPid() {
    m_armPidEnable = false;
    m_armRightMotor.set(0);
  }

  public void updateJointSoftwarePidControllerValues() {
    double p, i, d, tolerance;
    p = TestingDashboard.getInstance().getNumber(m_arm, "TargetArmP");
    i = TestingDashboard.getInstance().getNumber(m_arm, "TargetArmI");
    d = TestingDashboard.getInstance().getNumber(m_arm, "TargetArmD");
    tolerance = TestingDashboard.getInstance().getNumber(m_arm, "TargetArmTolerance");
    m_armPid.setP(p);
    m_armPid.setI(i);
    m_armPid.setD(d);
    m_armPid.setTolerance(tolerance);
    m_armPid.setSetpoint(m_armTargetAngle);
  }

  public void controlJointsWithSoftwarePidControl() {
    updateJointSoftwarePidControllerValues();

    // Do nothing if Arm PID control is not enabled
    if (!m_armPidEnable) {
      return;
    }

    double power = m_armPid.calculate(getArmAngle(), m_armTargetAngle);
    power = MathUtil.clamp(power, -Constants.ManipulatorConstants.kArmMaxPower, Constants.ManipulatorConstants.kArmMaxPower);
    m_armRightMotor.set(power);
  }

  public void updatePidEnableFlags() {
    if (m_armPidEnable) {
      TestingDashboard.getInstance().updateString(m_arm, "ArmSoftwarePidEnable", "Enabled");
    } else {
      TestingDashboard.getInstance().updateString(m_arm, "ArmSoftwarePidEnable", "Disabled");
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    TestingDashboard.getInstance().updateNumber(m_arm, "ArmCurrentR", m_armRightMotor.getOutputCurrent());
    TestingDashboard.getInstance().updateNumber(m_arm, "ArmCurrentL", m_armLeftMotor.getOutputCurrent());
    TestingDashboard.getInstance().updateNumber(m_arm, "ArmEncoderPulses", m_armEncoder.getPosition());
    TestingDashboard.getInstance().updateNumber(m_arm, "ArmAngle", getArmAngle());

    updatePidEnableFlags();
    controlJointsWithSoftwarePidControl();
  }
}
