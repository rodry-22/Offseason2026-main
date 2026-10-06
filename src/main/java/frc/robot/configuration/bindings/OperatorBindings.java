package frc.robot.configuration.bindings;

import java.util.function.DoubleSupplier;

import com.stzteam.mars.operator.ControllerOI;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.configuration.constants.moduleconstants.Dumperconstants;
import frc.robot.configuration.constants.moduleconstants.IntakeConstants;
import frc.robot.configuration.constants.moduleconstants.flywheelsConstants.shooterWheelsConstants;
import frc.robot.modules.superstructure.composite.Superstructure;
import frc.robot.modules.superstructure.modules.DumperModule.DumperIO.DumperMODE;
import frc.robot.requests.DumperRequestFactory;
import frc.robot.requests.IntakeRequestFactory;
import com.stzteam.mars.models.containers.Binding;

public class OperatorBindings implements Binding {

    private final ControllerOI  operator;

    private final Superstructure superstructure;
    private final DoubleSupplier distanceToHub; // m, de la pose fusionada con Limelight

      private final double DEADBAND = 0.1;

    // (constantes de prueba comentadas; las del disparo viven en flywheelsConstants / IndexerConstants)
    // private static final double SHOOT_ANGLE = 20.0;
    // private static final double SHOOT_RPM = 3000.0;
    // private static final double TEST_SHOOTER_SPEED = 1;
    // private static final double TEST_PROCESS_SPEED = 1;

    public OperatorBindings(ControllerOI operator, Superstructure superstructure, DoubleSupplier distanceToHub) {
        this.operator = operator;
        this.superstructure = superstructure;
        this.distanceToHub = distanceToHub;
    }

    public static OperatorBindings create(ControllerOI operator, Superstructure ss, DoubleSupplier distanceToHub) {
    return new OperatorBindings(operator, ss, distanceToHub);
  }

    @Override
    public void bind() {

      var buttons = operator.getActionButtons();
      var bumpers = operator.getBumpers();
      // var driverSystem = operator.getSystemTriggers();
      var leftStick = operator.getLeftStick();
      var rightStick = operator.getRightStick();
      var triggers = operator.getAnalogTriggers();
      var pov = operator.getDPadTriggers();

      Trigger rightStickXTrigger =
          new Trigger(() -> Math.abs(rightStick.x().getAsDouble()) > DEADBAND);
      Trigger rightStickYTrigger =
          new Trigger(() -> Math.abs(rightStick.y().getAsDouble()) > DEADBAND);
      Trigger leftStickXTrigger = new Trigger(() -> Math.abs(leftStick.x().getAsDouble()) > DEADBAND);
      Trigger leftStickYTrigger = new Trigger(() -> Math.abs(leftStick.y().getAsDouble()) > DEADBAND);

      // ---- comandos de prueba anteriores (comentados) ----
      // bumpers.right().whileTrue(superstructure.ProcessSpeed(TEST_PROCESS_SPEED));
      // bumpers.left().whileTrue(superstructure.spinShooter(TEST_SHOOTER_SPEED));
      // triggers.right().whileTrue(superstructure.shoot(SHOOT_ANGLE, SHOOT_RPM));

      // ---- SHOOT: flywheels con FF + PID; al llegar al RPM, arrancan los rollers ----
      // Trigger derecho: RPM segun la distancia al hub (Limelight / pose).
      triggers.right().whileTrue(superstructure.shootAtDistance(distanceToHub));
      // Trigger izquierdo: RPM fijo (shooterWheelsConstants.kShootRPM = 5000).
      triggers.left().whileTrue(superstructure.shoot(shooterWheelsConstants.kShootRPM));

      // ---- PRUEBAS de alcance (temporales) ----
      // A: flywheels al maximo (duty 100%). OJO: con Dumperconstants.kTuningMode = true, A (bottom)
      // tambien manda el dumper a 5 deg (esta mas abajo, en el bloque de calibracion).
      buttons.bottom().whileTrue(superstructure.spinShooterMax());
      // Bumper derecho: rollers + index a 3000 RPM.
      bumpers.right().whileTrue(superstructure.indexerTestRPM());

      // Intake: el trigger izquierdo ahora es del disparo. PENDIENTE reasignar a otro boton.
      // triggers.left().whileTrue(superstructure.intakeBalls()).onFalse(superstructure.retractIntake());

      // ---------------- CALIBRACION DEL INTAKE (temporal, solo con kAngulatorTuningMode) ----------------
      if (IntakeConstants.kAngulatorTuningMode) {
        var intake = superstructure.getIntake();
        // Stick derecho (Y) = voltaje manual +-3 V al angulador. Si va al reves, invierte el signo.
        rightStickYTrigger.whileTrue(
            intake.setControl(
                () -> IntakeRequestFactory.setAngulatorVolts().withVolts(rightStick.y().getAsDouble() * 3.0)));
      }

      // ---------------- CALIBRACION DEL DUMPER (temporal, solo con kTuningMode) ----------------
      if (Dumperconstants.kTuningMode) {
        var dumper = superstructure.getDumper();
        // Stick izquierdo (Y) = voltaje manual +-3 V. Sirve para medir kG/kS. Si va al reves, invierte el signo.
        leftStickYTrigger.whileTrue(
            dumper.setControl(
                () -> DumperRequestFactory.moveVoltage().withvolVolts(leftStick.y().getAsDouble() * 3.0)));
        // Escalon de posicion para ver la respuesta del PID en AdvantageScope.
        buttons.top().onTrue(dumper.setAngle(30.0, DumperMODE.kFRONT, Dumperconstants.kToleranceDeg));
        buttons.bottom().onTrue(dumper.setAngle(5.0, DumperMODE.kFRONT, Dumperconstants.kToleranceDeg));
      }

      // triggers.left().whileTrue(superstructure.intake());
      // buttons.bottom().onTrue(superstructure.stopAll());
   }


          



}