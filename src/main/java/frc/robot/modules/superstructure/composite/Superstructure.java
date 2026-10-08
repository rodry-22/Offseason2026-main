package frc.robot.modules.superstructure.composite;


import com.stzteam.mars.models.SubsystemBuilder;
import com.stzteam.mars.models.multimodules.CompositeSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.configuration.KeyManager;
import frc.robot.configuration.constants.Constants;
import frc.robot.configuration.constants.moduleconstants.Dumperconstants;
import frc.robot.configuration.constants.moduleconstants.IndexerConstants;
import frc.robot.configuration.constants.moduleconstants.IntakeConstants;
import frc.robot.configuration.constants.moduleconstants.flywheelsConstants.shooterWheelsConstants;
import frc.robot.configuration.constants.moduleconstants.flywheelsConstants.shooterWheelsConstants.IntakeWheelsConstants;
import frc.robot.modules.superstructure.modules.DumperModule.Dumper;
import frc.robot.modules.superstructure.modules.DumperModule.DumperIO.DumperMODE;
import frc.robot.modules.superstructure.modules.FlywheelsModule.flywheels;
import frc.robot.modules.superstructure.modules.IndexerModule.Indexer;
import frc.robot.modules.superstructure.modules.IntakeModule.Intake;
import frc.robot.modules.superstructure.modules.IntakeModule.IntakeIO.IntakeMODE;
import frc.robot.requests.DumperRequestFactory;
import frc.robot.requests.FlywheelsRequestFactory;
import frc.robot.requests.IndexerRequestFactory;

public class Superstructure extends CompositeSubsystem<SuperstructureData, SuperstructureIO>{

    private static final double intakeVolts = -8;
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

    public Intake getIntake(){
      return getSubsystem(KeyManager.INTAKE_KEY);
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
  return intakeWheels.setControl(
      () -> FlywheelsRequestFactory.setRPM().toRPM(IntakeWheelsConstants.kIntakeRPMeat));
}

public Command intakewheelsOut() {
  flywheels intakeWheels = getFlywheelsIntake();
  return intakeWheels.setControl(
      () -> FlywheelsRequestFactory.setRPM().toRPM(IntakeWheelsConstants.kIntakeRPMvomiatr));
}



//   public Command spinShooterRPM(double rpm){
//      flywheels flywheelShooter = getFlywheelsShooter();

//      return flywheelShooter.setControl(() -> FlywheelsRequestFactory.setRPM().toRPM(rpm));
//    }



//    public Command Process(){
//      Indexer index = getIndexer();

//      // Voltaje en vez de RPM: el SparkMax no tiene PID configurado (kP = 0),
//      // asi que un setpoint de velocidad no mueve el motor.
//      return index.setControl(
//          () -> IndexerRequestFactory.processing().withRollers(feedVolts).withIndex(feedVolts));
//    }

//    // Sin PID: mueve el shooter en duty cycle (-1.0 a 1.0)
//    public Command spinShooter(double speed){
//      flywheels flywheelShooter = getFlywheelsShooter();

//      return flywheelShooter.setControl(() -> FlywheelsRequestFactory.moveSpeed().whiSpeed(speed));
//    }

//    // Process sin PID, en duty cycle. Negativo = feedear (positivo expulsaba)
//    public Command ProcessSpeed(double speed){
//      Indexer index = getIndexer();

//      return index.setControl(
//          () -> IndexerRequestFactory.processingSpeed().withRollers(-speed).withIndex(-speed));
//    }


//    // ---------------------------------- DISPARO ----------------------------------
//    private static final double kFeedSpeed = 1.0; // duty; ProcessSpeed ya invierte el signo para alimentar

//    /** Disparo con angulo y RPM fijos. Util para medir puntos de la tabla de tiro. */
//    public Command shoot(double dumperAngleDeg, double shooterRPM) {
//      return shootInternal(() -> dumperAngleDeg, () -> shooterRPM);
//    }

//    /** Disparo automatico: angulo y RPM salen de las tablas de Constants segun la distancia al hub. */
//    public Command shootAtDistance(DoubleSupplier distanceMeters) {
//      return shootInternal(
//          () -> Constants.DUMPER_ANGLE_MAP.get(distanceMeters.getAsDouble()),
//          () -> Constants.SHOOTER_RPM_MAP.get(distanceMeters.getAsDouble()));
//    }

//    private Command shootInternal(DoubleSupplier angleDeg, DoubleSupplier rpm) {
//      flywheels shooter = getFlywheelsShooter();
//      Dumper dumper = getDumper();
//      Indexer index = getIndexer();

//      DoubleSupplier clampedAngle =
//          () ->
//              MathUtil.clamp(
//                  angleDeg.getAsDouble(),
//                  Dumperconstants.kLowerLimitDeg,
//                  Dumperconstants.kUpperLimitDeg);

//      // "Listo" se calcula contra el objetivo deseado, no contra inputs.TargetAngle/targetRPM:
//      // esos valen 0 antes del primer ciclo y darian un falso "en objetivo" que alimenta de golpe.
//      BooleanSupplier ready =
//          () ->
//              Math.abs(shooter.getState().velocityRPM - rpm.getAsDouble())
//                      <= Constants.FLYWHEEL_TOLERANCE
//                  && Math.abs(dumper.getState().position - clampedAngle.getAsDouble())
//                      <= Dumperconstants.kToleranceDeg;
//      Trigger readyStable = new Trigger(ready).debounce(0.15);

//      // Shooter y dumper se mantienen activos mientras el comando viva (whileTrue);
//      // el indexer solo alimenta cuando ambos estan en objetivo de forma estable.
//      return Commands.parallel(
//          shooter.setControl(() -> FlywheelsRequestFactory.setRPM().toRPM(rpm.getAsDouble())),
//          dumper.setControl(
//              () ->
//                  DumperRequestFactory.setAngle()
//                      .withAngle(clampedAngle.getAsDouble())
//                      .withMode(DumperMODE.kFRONT)
//                      .Tolerance(Dumperconstants.kToleranceDeg)),
//          Commands.sequence(
//              Commands.waitUntil(readyStable),
//              index.setControl(
//                  () ->
//                      IndexerRequestFactory.processingSpeed()
//                          .withRollers(-kFeedSpeed)
//                          .withIndex(-kFeedSpeed))));
//    }

    // ---------------------------------- SHOOT ----------------------------------
    // Flywheels con FF + PID (VelocityVoltage del Kraken). Cuando llegan al RPM, los rollers
    // arrancan a IndexerConstants.kRollerFeedRPM. Usar con whileTrue: al soltar todo se apaga solo.

    /** Disparo a un RPM fijo (trigger izquierdo: shooterWheelsConstants.kShootRPM). */
    public Command shoot(double rpm) {
      return shootInternal(() -> rpm);
    }

    /** Disparo con el RPM de la tabla segun la distancia al hub (pose fusionada con la Limelight). */
    public Command shootAtDistance(DoubleSupplier distanceMeters) {
      return shootInternal(() -> Constants.SHOOTER_RPM_MAP.get(distanceMeters.getAsDouble()));
    }

    private Command shootInternal(DoubleSupplier rpm) {
      flywheels shooter = getFlywheelsShooter();
      //Indexer index = getIndexer();

      // Nunca pedir mas RPM de los que se alcanzan (kMaxWheelRPM) ni NaN (la distancia al hub
      // puede salir NaN si la pose no es valida).
      DoubleSupplier target =
          () -> {
            double r = rpm.getAsDouble();
            return Double.isNaN(r) ? 0.0 : MathUtil.clamp(r, 0.0, shooterWheelsConstants.kMaxWheelRPM);
          };

      // "Listo" = la rueda ya llego al RPM (solo se exige llegar por abajo; pasarse no importa).
      // Se compara contra el RPM deseado, no contra inputs.targetRPM (vale 0 antes del primer ciclo).
      BooleanSupplier atSpeed =
          () ->
              shooter.getState().velocityRPM
                  >= target.getAsDouble() - shooterWheelsConstants.kRPMTolerance;
      Trigger readyStable = new Trigger(atSpeed).debounce(shooterWheelsConstants.kReadyDebounceSec);

      return Commands.parallel(
          shooter.setControl(() -> FlywheelsRequestFactory.setRPM().toRPM(target.getAsDouble())),
          Commands.sequence(
              // Espera a que la rueda llegue, pero nunca mas de kSpinUpTimeoutSec: si el RPM no se
              // alcanza (bateria baja, carga), igual alimenta en vez de quedarse esperando.
              Commands.waitUntil(readyStable).withTimeout(shooterWheelsConstants.kSpinUpTimeoutSec)
              // Rollers + index a su RPM de disparo (request processingRPM de IndexerRequest).
            //   index.setControl(
            //       () ->
            //           IndexerRequestFactory.processingRPM()
            //               .withRollers(IndexerConstants.kRollerFeedRPM)
            //               .withIndex(IndexerConstants.kIndexFeedRPM))
            ));
    }

    // ---------------------------------- PRUEBAS (temporales) ----------------------------------
    /** PRUEBA de alcance: flywheels al 100% de duty cycle, sin PID. Usar con whileTrue. */
    public Command spinShooterMax() {
      flywheels shooter = getFlywheelsShooter();

      return shooter.setControl(
          () -> FlywheelsRequestFactory.moveSpeed().whiSpeed(shooterWheelsConstants.kMaxDuty));
    }

    /** PRUEBA: rollers e index a 3000 RPM con FF + PID del SparkMax. Usar con whileTrue. */
    public Command indexerTestRPM() {
      Indexer index = getIndexer();

      return index.setControl(
          () ->
              IndexerRequestFactory.processingRPM()
                  .withRollers(IndexerConstants.kRollerFeedRPM)
                  .withIndex(IndexerConstants.kIndexFeedRPM));
    }

    /** PRUEBA: rollers e index por CORRIENTE (kRollerFeedAmps / kIndexFeedAmps, 40 A). whileTrue. */
    public Command indexerTestAmps() {
      Indexer index = getIndexer();

      return index.setControl(
          () ->
              IndexerRequestFactory.processingAmps()
                  .withRollers(IndexerConstants.kRollerFeedAmps)
                  .withIndex(IndexerConstants.kIndexFeedAmps));
    }

    // ---------------------------------- INTAKE ----------------------------------
    /**
     * Baja el brazo y, SOLO cuando llego abajo, enciende las flywheels del intake.
     * Usar con whileTrue: al soltar se cancela y las flywheels se apagan solas (default idle).
     * Para que el brazo suba al soltar, encadenar .onFalse(retractIntake()).
     */
    public Command intakeBalls() {
      Intake intake = getIntake();
      flywheels intakeWheels = getFlywheelsIntake();

      Trigger deployed =
          new Trigger(
                  () ->
                      Math.abs(intake.getState().angulatorPosition - IntakeConstants.kDeployAngleDeg)
                          <= IntakeConstants.toleranceDegrees)
              .debounce(0.1);

      return Commands.parallel(
          intake.setAngulatorPosition(IntakeConstants.kDeployAngleDeg, IntakeMODE.kFRONT),
          Commands.sequence(
              Commands.waitUntil(deployed),
              intakeWheels.setControl(
                  () -> FlywheelsRequestFactory.moveVoltage().whithVolts(intakeVolts))));
    }

    /** Baja el brazo a la posicion de recoleccion y termina (despues el default lo mantiene ahi). */
    public Command Intakedown() {
      Intake intake = getIntake();

      return intake
          .setAngulatorPosition(IntakeConstants.kDeployAngleDeg, IntakeMODE.kFRONT)
          .until(
              () ->
                  Math.abs(intake.getState().angulatorPosition - IntakeConstants.kDeployAngleDeg)
                      <= IntakeConstants.toleranceDegrees)
          .withTimeout(5.0);
    }

    /** Sube el brazo a la posicion guardada y termina (despues el default lo mantiene ahi). */
    public Command retractIntake() {
      Intake intake = getIntake();

      return intake
          .setAngulatorPosition(IntakeConstants.kStowAngleDeg, IntakeMODE.kBACK)
          .until(
              () -> 
                  Math.abs(intake.getState().angulatorPosition - IntakeConstants.kStowAngleDeg)
                      <= IntakeConstants.toleranceDegrees)
          .withTimeout(5.0);
    }

    public Command stopAll() {

    Dumper dumper = getDumper();
    flywheels flywheel = getFlywheelsIntake();
    flywheels flywheelout = getFlywheelsShooter();
    Indexer index = getIndexer();
    Intake intake = getIntake();

    return Commands.parallel(
        dumper.sotop(),
        index.idleIndexer(),
        flywheel.runRequest(() -> FlywheelsRequestFactory.idleIntake()),
        flywheelout.runRequest(() -> FlywheelsRequestFactory.idleOutake()),
        intake.stop()  
    );
    
 }
 
  public SuperstructureData getState() {
    return inputs;
  }
    


@Override
  public void absolutePeriodic(SuperstructureData inputs) {
  }
}