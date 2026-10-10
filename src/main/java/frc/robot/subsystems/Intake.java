package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.SimpleDifferentialMechanism;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.utils.HolonomicController;

public class Intake implements Subsystem{

    
    private final TalonFX intakeMotor = new TalonFX(0);
    private final MotionMagicVelocityVoltage intakeVoltage = new MotionMagicVelocityVoltage(0);

    private final SysIdRoutine _intakeRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> setControl(intakeVoltage.withVelocity(output)), null, this));
      
    
    public Intake() {
        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.14;
        slot0Configs.kV = 0.939;
        slot0Configs.kA = 0.002;
        slot0Configs.kG = 0.435;
        slot0Configs.kP = 28;
        slot0Configs.kD = 0.2;

        var motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = 80;
        motionMagicConfigs.MotionMagicAcceleration = 160;
        motionMagicConfigs.MotionMagicJerk = 1600;

        intakeMotor.getConfigurator().apply(talonFXConfigs);
    }

    public void intakeSpin(double velocity) {
        intakeMotor.setControl(intakeVoltage.withVelocity(velocity));
    }

    public Command intakeCommand(double velocity) {
        return run(
            () -> {
                intakeSpin(velocity);
            }
        );
    }






}
