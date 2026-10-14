package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Intake implements Subsystem {  // Use AdvancededSubsystem instead of Subsystem

  private final TalonFX feedMotor = new TalonFX(Constants.MotorConstants.kfeedMotor);
  private final TalonFX updownMotor = new TalonFX(Constants.MotorConstants.kupdownMotor); //rename it to actutator motor

  private final VelocityVoltage intakeVoltageSetter = new VelocityVoltage(0); //use VelocityVoltage for the rollers
  private final MotionMagicVoltage updownVoltageSetter = new MotionMagicVoltage(0);

  private final double updownPositionGetter = updownMotor.getPosition().getValueAsDouble();


  private final VoltageOut sysIdVoltage = new VoltageOut(0);

  private final SysIdRoutine _feedRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> feedMotor.setControl(sysIdVoltage.withOutput(output.in(Volts))), 
              null, 
              this
            )
        );

    private final SysIdRoutine _updownRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> updownMotor.setControl(sysIdVoltage.withOutput(output.in(Volts))), 
              null, 
              this
            )
        );

  public Intake() {

    feedMotor.getConfigurator().apply(Constants.Gains.kfeedGains);

    updownMotor.getConfigurator().apply(Constants.Gains.kupdownGains);
    updownMotor.getConfigurator().apply(Constants.Gains.kmotionMagicUpdownConfig);

    updownMotor.setPosition(0);
  }

    public double getUpdownPosition() {
        return updownMotor.getPosition().getValueAsDouble(); // rotations
    }

    public boolean isAtPosition(double target) {
        return Math.abs(getUpdownPosition() - target) < Constants.kupdownTolerance;
    }

    public void intake() {
        if (!isAtPosition(Constants.kintakeDownPosition)) {
            updownMotor.setControl(updownVoltageSetter.withPosition(Constants.kintakeDownPosition));
            feedMotor.setControl(intakeVoltageSetter.withVelocity(Constants.kfeedVelocity));
        }
    }

  public Command intakeCommand() {
    return run(
        () -> {
          intake();
        });
  }
}
