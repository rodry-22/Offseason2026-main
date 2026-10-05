package frc.robot.configuration.bindings;

import com.stzteam.mars.operator.ControllerOI;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.configuration.constants.moduleconstants.Dumperconstants;
import frc.robot.modules.superstructure.composite.Superstructure;
import frc.robot.modules.superstructure.modules.DumperModule.DumperIO.DumperMODE;
import frc.robot.requests.DumperRequestFactory;
import com.stzteam.mars.models.containers.Binding;

public class OperatorBindings implements Binding {

    private final ControllerOI  operator;

    private final Superstructure superstructure;

      private final double DEADBAND = 0.1;

    private static final double SHOOT_ANGLE = 20.0;
    private static final double SHOOT_RPM = 3000.0;
    private static final double TEST_SHOOTER_SPEED = 1; // 70% de potencia
    private static final double TEST_PROCESS_SPEED = 1; // 40% de potencia

    public OperatorBindings(ControllerOI operator, Superstructure superstructure) {
        this.operator = operator;
        this.superstructure = superstructure;
    }

    public static OperatorBindings create(ControllerOI operator, Superstructure ss) {
    return new OperatorBindings(operator, ss);
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


      

      // Por ahora solo probamos el Process del indexer
      bumpers.right().whileTrue(superstructure.ProcessSpeed(TEST_PROCESS_SPEED));

      // Prueba de flywheels del shooter con setSpeed (sin PID)
      bumpers.left().whileTrue(superstructure.spinShooter(TEST_SHOOTER_SPEED));

      // Disparo con valores fijos (SHOOT_ANGLE / SHOOT_RPM). Activar SOLO despues de calibrar el
      // dumper (pasos 0-3 de la guia) y de verificar que el shooter gira en el sentido correcto.
      triggers.right().whileTrue(superstructure.shoot(SHOOT_ANGLE, SHOOT_RPM));

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