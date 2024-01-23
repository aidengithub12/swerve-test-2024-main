// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
// flywheel
package frc.robot.subsystems.Intake.Intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class IntakeIOSparkMax implements IntakeIO {
  private static final double GEAR_RATIO = 1.5;

  private final CANSparkMax Motor1 = new CANSparkMax(0, MotorType.kBrushless);
  private static final CANSparkMax Motor2 = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax Motor3 = new CANSparkMax(2, MotorType.kBrushless);
  static final RelativeEncoder encoder = Motor2.getEncoder();

  public void ShooterIOSparkMax() {
    Motor1.restoreFactoryDefaults();
    Motor2.restoreFactoryDefaults();
    Motor3.restoreFactoryDefaults();

    Motor1.setCANTimeout(250);
    Motor2.setCANTimeout(250);
    Motor3.setCANTimeout(250);
    Motor1.setInverted(false);

    Motor1.enableVoltageCompensation(12.0);
    Motor1.setSmartCurrentLimit(30);
    Motor2.enableVoltageCompensation(12);
    Motor2.setSmartCurrentLimit(30);
    Motor3.enableVoltageCompensation(12);
    Motor3.setSmartCurrentLimit(30);

    Motor1.burnFlash();
    Motor2.burnFlash();
    Motor3.burnFlash();
  }

  public void updateInputs(IntakeIOInputs inputs) {
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts = Motor1.getAppliedOutput() * Motor1.getBusVoltage();
    inputs.currentAmps = new double[] {Motor1.getOutputCurrent(), Motor2.getOutputCurrent()};
  }

  public double getVelocityRPM() {
    return encoder.getVelocity();
  }

  public double getCharacterizationVelocity() {
    return encoder.getVelocity();
  }

  @Override
  public void setVoltage(double volts) {
    Motor1.setVoltage(volts);
    Motor2.setVoltage(volts);
    Motor3.setVoltage(volts);
  }

  @Override
  public void stop() {
    Motor1.stopMotor();
    Motor2.stopMotor();
    Motor3.stopMotor();
  }
}
