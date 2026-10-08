package frc.robot.modules.superstructure.modules.FlywheelsModule;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.robot.configuration.constants.moduleconstants.flywheelsConstants.shooterWheelsConstants;

public class FlyWheelIOKrakenIntake implements flywheelsIO {

    private TalonFX leaderShooter, followerShooter;
    private TalonFXConfiguration leaderConfig, followerConfig;
    private TalonFXConfigurator leaderConfigurator, followerConfigurator;

    private VelocityVoltage velocityRequest;
    private double velocityTarget;

    public FlyWheelIOKrakenIntake(){
        leaderShooter = new TalonFX(shooterWheelsConstants.shooterLeaderID, CANBus.roboRIO());
        followerShooter = new TalonFX(shooterWheelsConstants.shooterFollowerID, CANBus.roboRIO());

        leaderConfig = new TalonFXConfiguration();
        followerConfig = new TalonFXConfiguration();

        leaderConfigurator = leaderShooter.getConfigurator();
        followerConfigurator = followerShooter.getConfigurator();

        velocityRequest = new VelocityVoltage(0);

        followerShooter.setControl(new Follower(shooterWheelsConstants.shooterLeaderID, MotorAlignmentValue.Opposed));

        configMotor();
    }

    public void configMotor(){
        var limitConfigs = leaderConfig.CurrentLimits;

    limitConfigs.SupplyCurrentLimit = shooterWheelsConstants.SupplyCurrentLimit;
    limitConfigs.SupplyCurrentLimitEnable = shooterWheelsConstants.SupplyCurrentLimitEnable;

    limitConfigs.StatorCurrentLimit = shooterWheelsConstants.StatorCurrentLimit;
    limitConfigs.StatorCurrentLimitEnable = shooterWheelsConstants.StatorCurrentLimitEnable;

    // Sentido de giro: con kShooterInverted el disparo queda en sentido positivo (ver constantes).
    leaderConfig.MotorOutput.Inverted = shooterWheelsConstants.kShooterInverted;
    followerConfig.MotorOutput.Inverted = shooterWheelsConstants.kShooterInverted;

    var slot0Configs = leaderConfig.Slot0;

    slot0Configs.kS = shooterWheelsConstants.kS;
    slot0Configs.kV = shooterWheelsConstants.kV;
    slot0Configs.kP = shooterWheelsConstants.kP;
    slot0Configs.kI = shooterWheelsConstants.kI;
    slot0Configs.kD = shooterWheelsConstants.kD;

    leaderConfigurator.apply(leaderConfig);
    followerConfigurator.apply(followerConfig);

    leaderConfigurator.apply(limitConfigs);
    followerConfigurator.apply(limitConfigs);
    }
    
    @Override 
    public void updateInputs(FlyWheelsInputs inputs){
        double motorRPM = leaderShooter.getVelocity().getValueAsDouble() * 60.0; //Convertir RPS A RPM
        inputs.motorRPM = motorRPM;
        inputs.velocityRPM = motorRPM / shooterWheelsConstants.kGearRatio; // RPM de la RUEDA

        // Follower: si su magnitud/corriente no se parece a la del lider, los motores se pelean.
        inputs.followerRPM = followerShooter.getVelocity().getValueAsDouble() * 60.0;
        inputs.followerCurrent = followerShooter.getStatorCurrent().getValueAsDouble();

        inputs.appliedVolts = leaderShooter.getMotorVoltage().getValueAsDouble();
        inputs.targetRPM = this.velocityTarget;

        inputs.current = leaderShooter.getStatorCurrent().getValueAsDouble();
    }

    @Override 
    public void setTargetRPM(double RPM){
        this.velocityTarget = RPM;
        // RPM es de la RUEDA; el motor debe girar kGearRatio veces mas rapido (las ganancias siguen en unidades del motor).
        leaderShooter.setControl(
            velocityRequest.withVelocity(RPM * shooterWheelsConstants.kGearRatio / 60.0).withSlot(0));   
    }

    @Override 
    public void applyOutput(double volts){
        leaderShooter.setVoltage(volts);
    }

    @Override
    public void setSpeed(double speed){
        leaderShooter.set(speed);
    }
}