package frc.robot.modules.superstructure.composite;


import com.stzteam.mars.models.SubsystemBuilder;
import com.stzteam.mars.models.multimodules.CompositeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.configuration.KeyManager;
import frc.robot.configuration.constants.Constants;
import frc.robot.modules.superstructure.modules.DumperModule.Dumper;
import frc.robot.modules.superstructure.modules.DumperModule.DumperIO.DumperMODE;
import frc.robot.modules.superstructure.modules.FlywheelsModule.flywheels;
import frc.robot.modules.superstructure.modules.IndexerModule.Indexer;
import frc.robot.requests.DumperRequestFactory;
import frc.robot.requests.FlywheelsRequestFactory;
import frc.robot.requests.IndexerRequestFactory;

public class Superstructure extends CompositeSubsystem<SuperstructureData, SuperstructureIO>{

    private static final double intakeVolts = -5;
    private static final double feedVolts = 12;


    public Superstructure(SubsystemBuilder<SuperstructureData, SuperstructureIO> builder){
        super(builder);
    }

    public Dumper getDumper(){
        return getSubsystem(KeyManager.DUMPER_KEY);
    }

    public Indexer getIndexer(){
        return getSubsystem(KeyManager.INDEXER_KEY);
    }

    public flywheels getFlywheelsShooter(){
      return getSubsystem(KeyManager.FLYWHEELS_SHOOTER_KEY);
    }

    public flywheels getFlywheelsIntake(){
      return getSubsystem(KeyManager.FLYWHEELS_INTAKE_KEY);
    }

    //----------------------------------COMMANDS-------------------------------------------------------------
/* 
    public Command shoot(double turretAngle, double armAngle, double shooterRPM) {
      flywheels flywheelShooter = getFlywheelsShooter();
      Indexer index = getIndexer();
      Dumper dumper = getDumper();
      flywheels intakeWheels = getFlywheelsIntake();

      // Shooter y dumper se quedan activos todo el comando; el indexer e intake
      // solo empiezan a alimentar cuando el shooter llega a las RPM objetivo.
      return Commands.parallel(
          flywheelShooter.setControl(() -> FlywheelsRequestFactory.setRPM().toRPM(shooterRPM)),
          dumper.setControl(
              () -> DumperRequestFactory.setAngle().withAngle(armAngle).withMode(DumperMODE.kFRONT)),
          Commands.sequence(
              Commands.waitUntil(() -> flywheelShooter.isAtTarget(Constants.FLYWHEEL_TOLERANCE)),
              Commands.parallel(
                  index.setControl(
                      () -> IndexerRequestFactory.processing().withRollers(feedVolts).withIndex(feedVolts)),
                  intakeWheels.setControl(
                      () -> FlywheelsRequestFactory.moveVoltage().whithVolts(intakeVolts)))));
  }
                      */


//MOVE WHEELS INTAKE
    public Command intakewheels() {
      flywheels intakeWheels = getFlywheelsIntake();

      return Commands.parallel(
          intakeWheels.setControl(() -> FlywheelsRequestFactory.moveVoltage().whithVolts(intakeVolts)));
  }



    public Command Process(){
      Indexer index = getIndexer();

      // Voltaje en vez de RPM: el SparkMax no tiene PID configurado (kP = 0),
      // asi que un setpoint de velocidad no mueve el motor.
      return index.setControl(
          () -> IndexerRequestFactory.processing().withRollers(feedVolts).withIndex(feedVolts));
    }

    // Sin PID: mueve el shooter en duty cycle (-1.0 a 1.0)
    public Command spinShooter(double speed){
      flywheels flywheelShooter = getFlywheelsShooter();

      return flywheelShooter.setControl(() -> FlywheelsRequestFactory.moveSpeed().whiSpeed(speed));
    }

    // Process sin PID, en duty cycle. Negativo = feedear (positivo expulsaba)
    public Command ProcessSpeed(double speed){
      Indexer index = getIndexer();

      return index.setControl(
          () -> IndexerRequestFactory.processingSpeed().withRollers(-speed).withIndex(-speed));
    }

    public Command stopAll() {

    Dumper dumper = getDumper();
    flywheels flywheel = getFlywheelsIntake();
    flywheels flywheelout = getFlywheelsShooter();
    Indexer index = getIndexer();
    //Intake intake = getIntake();

    return Commands.parallel(
        dumper.stop(),
        index.idleIndexer(),
        flywheel.runRequest(() -> FlywheelsRequestFactory.idleIntake()),
        flywheelout.runRequest(() -> FlywheelsRequestFactory.idleOutake())

        
    );
    
 }
 /* 
  @Override
  public SuperstructureData getState() {
    return inputs;
  }
    */


@Override
  public void absolutePeriodic(SuperstructureData inputs) {
  }
}
