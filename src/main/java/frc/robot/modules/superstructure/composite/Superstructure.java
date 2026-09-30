package frc.robot.modules.superstructure.composite;

import static edu.wpi.first.units.Units.RPM;

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
import frc.robot.requests.IndexerRequest;
import frc.robot.requests.IndexerRequestFactory;

public class Superstructure extends CompositeSubsystem<SuperstructureData, SuperstructureIO>{

    private static final double intakeVolts = -5;


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

    public Command shoot(double turretAngle, double armAngle, double shooterRPM) {
      flywheels flywheelShooter = getFlywheelsShooter();
      Indexer index = getIndexer();
      Dumper dumper = getDumper();
      flywheels intakeWheels = getFlywheelsIntake();

      return Commands.sequence(
        Commands.parallel(
                flywheelShooter.runRequest(
                    () ->
                        FlywheelsRequestFactory.setRPM()
                            .toRPM(shooterRPM)
                            ),
                dumper.setControl(
                    () -> DumperRequestFactory.setAngle().withAngle(armAngle).withMode(DumperMODE.kFRONT)))
                    .until(() -> flywheelShooter.isAtTarget(Constants.FLYWHEEL_TOLERANCE)),
        Commands.parallel(
                index.setControl(() -> IndexerRequestFactory.setRollers().withRPS(shooterRPM))),
                //index.setControl(
                 // () -> IndexerRequestFactory.moveVoltage().withRollers(12).withIndex(12)),

                intakeWheels.setControl(
                  ()-> FlywheelsRequestFactory.moveVoltage().whithVolts(intakeVolts)));
                
  }

    public Command Process(){
      Indexer index = getIndexer();

       return Commands.parallel(
        index.setControl(() -> IndexerRequestFactory.setRollers().withRPS(intakeVolts)));
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
