package frc.robot.modules.superstructure.modules.FlywheelsModule;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.configuration.constants.TunerConstants;
import frc.robot.configuration.constants.moduleconstants.flywheelsConstants.shooterWheelsConstants.IntakeWheelsConstants;

public class FlyWheelIOKrakenIntake implements flywheelsIO {

    private final TalonFX intakeFlyWheels;
  private TalonFXConfigurator FlyWheelsConfigurator;

  public FlyWheelIOKrakenIntake() {
    intakeFlyWheels = new TalonFX(IntakeWheelsConstants.IntakeWheels_ID, TunerConstants.kCANBus);
    FlyWheelsConfigurator = intakeFlyWheels.getConfigurator();

    configMotor();
    // optimizeCANBus();
  }

  public void configMotor() {
    var motorConfigs = new MotorOutputConfigs();

    motorConfigs.Inverted = IntakeWheelsConstants.invertedValue;
    motorConfigs.NeutralMode = NeutralModeValue.Brake;

    var limitConfigs = new CurrentLimitsConfigs();

    limitConfigs.StatorCurrentLimit = IntakeWheelsConstants.StatorCurrentLimit;
    limitConfigs.StatorCurrentLimitEnable = true;

    limitConfigs.SupplyCurrentLimit = IntakeWheelsConstants.SupplyCurrentLimit;
    limitConfigs.SupplyCurrentLimitEnable = true;

    intakeFlyWheels.getConfigurator().apply(limitConfigs);

    FlyWheelsConfigurator.refresh(motorConfigs);
    FlyWheelsConfigurator.apply(motorConfigs);
  }

  public void optimizeCANBus() {

    // 50 hz = 20ms
    intakeFlyWheels.getMotorVoltage().setUpdateFrequency(50);
    intakeFlyWheels.getVelocity().setUpdateFrequency(50);

    intakeFlyWheels.getPosition().setUpdateFrequency(0);
    intakeFlyWheels.getClosedLoopError().setUpdateFrequency(0);
    intakeFlyWheels.getClosedLoopDerivativeOutput().setUpdateFrequency(0);
    intakeFlyWheels.getClosedLoopProportionalOutput().setUpdateFrequency(0);
    intakeFlyWheels.getClosedLoopIntegratedOutput().setUpdateFrequency(0);

    intakeFlyWheels.optimizeBusUtilization();
  }

  @Override
  public void applyOutput(double volts) {
    intakeFlyWheels.setVoltage(volts);
  }

  @Override
  public void setSpeed(double speed) {
    intakeFlyWheels.set(speed);
  }

  @Override
  public void setTargetRPM(double RPM) {}

  @Override
  public void updateInputs(FlyWheelsInputs inputs) {
    inputs.appliedVolts = intakeFlyWheels.getMotorVoltage().getValueAsDouble();
    inputs.velocityRPM = intakeFlyWheels.getVelocity().getValueAsDouble();

    inputs.current = intakeFlyWheels.getStatorCurrent().getValueAsDouble();
  }
}