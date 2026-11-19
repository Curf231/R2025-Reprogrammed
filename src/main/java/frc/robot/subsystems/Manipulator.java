// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.AdvancedSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants.ManipulatorCoinstants;

public class Manipulator extends AdvancedSubsystem {
  /** Creates a new Wrist. */

  private final TalonFX _leftAlgaeMotor = new TalonFX(ManipulatorCoinstants._leftAlgaeMotorID);
  private final TalonFX _rightAlgaeMotor = new TalonFX(ManipulatorCoinstants._rightAlgaeMotor);

  private final DigitalInput _algeaBeambrake = new DigitalInput(ManipulatorCoinstants._algeaBeambrakeID);
  private final DigitalInput _coralBeambrake = new DigitalInput(ManipulatorCoinstants._coralBeambrake);

  private final BooleanEvent _coralSeen = new BooleanEvent(null, null); // second param should be this._coralBeambreak
  private final BooleanEvent _algeaSeen = new BooleanEvent(null, null);

  private final SysIdRoutine _LeftMotorRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(0), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> _leftAlgaeMotor.setControl(sysIdVoltage.withOutput(output.in(Volts))),
              null,
              this));

  private final SysIdRoutine _RightMotorRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(0), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> _rightAlgaeMotor.setControl(sysIdVoltage.withOutput(output.in(Volts))),
              null,
              this));


  public Manipulator() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
