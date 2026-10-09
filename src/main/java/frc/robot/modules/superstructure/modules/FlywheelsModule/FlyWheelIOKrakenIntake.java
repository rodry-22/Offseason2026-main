package frc.robot.modules.superstructure.modules.FlywheelsModule;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.configuration.constants.moduleconstants.flywheelsConstants.shooterWheelsConstants.IntakeWheelsConstants;

public class FlyWheelIOKrakenIntake implements flywheelsIO {

    // Un solo motor: el del intake (ID 14). ANTES usaba los IDs 19 y 20, que son los del SHOOTER.
    private TalonFX intakeMotor;
    private TalonFXConfiguration intakeConfig;
    private TalonFXConfigurator intakeConfigurator;

    private VelocityVoltage velocityRequest;
    private double velocityTarget;

    public FlyWheelIOKrakenIntake(){
        // CONFIRMAR: que el motor 14 este en el bus del roboRIO (mismo bus que el swerve ahora).
        intakeMotor = new TalonFX(IntakeWheelsConstants.IntakeWheels_ID, CANBus.roboRIO());

        intakeConfig = new TalonFXConfiguration();
        intakeConfigurator = intakeMotor.getConfigurator();

        velocityRequest = new VelocityVoltage(0);

        configMotor();
    }

    public void configMotor(){
        var limitConfigs = intakeConfig.CurrentLimits;

        limitConfigs.SupplyCurrentLimit = IntakeWheelsConstants.SupplyCurrentLimit;
        limitConfigs.SupplyCurrentLimitEnable = true;

        limitConfigs.StatorCurrentLimit = IntakeWheelsConstants.StatorCurrentLimit;
        limitConfigs.StatorCurrentLimitEnable = true;

        // Si el intake gira al reves, cambiar IntakeWheelsConstants.invertedValue.
        intakeConfig.MotorOutput.Inverted = IntakeWheelsConstants.invertedValue;

        // Ganancias PROPIAS del intake (antes se usaban las del shooter).
        var slot0Configs = intakeConfig.Slot0;
        slot0Configs.kS = IntakeWheelsConstants.kS;
        slot0Configs.kV = IntakeWheelsConstants.kV;
        slot0Configs.kP = IntakeWheelsConstants.kP;

        intakeConfigurator.apply(intakeConfig);
    }

    @Override
    public void updateInputs(FlyWheelsInputs inputs){
        double motorRPM = intakeMotor.getVelocity().getValueAsDouble() * 60.0; // RPS -> RPM
        inputs.motorRPM = motorRPM;
        inputs.velocityRPM = motorRPM; // sin reduccion conocida

        // El intake no tiene follower.
        inputs.followerRPM = 0;
        inputs.followerCurrent = 0;

        inputs.appliedVolts = intakeMotor.getMotorVoltage().getValueAsDouble();
        inputs.targetRPM = this.velocityTarget;

        inputs.current = intakeMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setTargetRPM(double RPM){
        this.velocityTarget = RPM;
        intakeMotor.setControl(velocityRequest.withVelocity(RPM / 60.0).withSlot(0));
    }

    @Override
    public void applyOutput(double volts){
        intakeMotor.setVoltage(volts);
    }

    @Override
    public void setSpeed(double speed){
        intakeMotor.set(speed);
    }
}