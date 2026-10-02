package frc.robot.tuning;

import static org.junit.jupiter.api.Assertions.assertFalse;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

/**
 * Autotune del PID de seguimiento de trayectoria de PathPlanner
 * ({@code PPHolonomicDriveController}) usado en {@link
 * frc.robot.modules.swerve.CommandSwerveDrivetrain#configurePathPlanner()}.
 *
 * <p>Corre una optimizacion Nelder-Mead sobre el modelo simulado de {@link
 * HolonomicPathFollowingSim} -- no sobre el robot real ni sobre paths
 * exportados de la GUI de PathPlanner. Lo que te da son ganancias de arranque
 * razonables para probar en el robot; siempre valida despues con un par de
 * autos reales (o en el simulador con {@code wpi.sim.addGui()}) antes de
 * competir, porque el modelo de friccion/retraso del chasis es una
 * aproximacion de primer orden, no el robot exacto.
 *
 * <p>Antes de correrlo, ajusta {@code translationTauSeconds},
 * {@code translationKsMetersPerSec}, {@code rotationTauSeconds} y {@code
 * rotationKsRadPerSec} en {@link HolonomicPathFollowingSim} con los valores
 * reales de tu analisis de SysId (ya tienen las rutinas corriendo en {@code
 * SysIdRoutineManager}; solo falta pasar el kS/kV/kA que salio del analisis).
 *
 * <p>Correr solo este test:
 * {@code ./gradlew test --tests "*PIDAutoTuneTest*" --info}
 * (el {@code --info} es para que Gradle no se trague los println).
 */
class PIDAutoTuneTest {

  @Test
  void autoTunePathFollowingPID() {
    // Escenarios representativos: linea recta larga, diagonal con giro de
    // 90 grados, y giro casi en el lugar con desplazamiento chico. Ajusta o
    // agrega los que se parezcan mas a tus autos reales de esta temporada.
    HolonomicPathFollowingSim.Scenario[] scenarios = {
      new HolonomicPathFollowingSim.Scenario(
          new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
          new Pose2d(3.0, 0, Rotation2d.fromDegrees(0))),
      new HolonomicPathFollowingSim.Scenario(
          new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
          new Pose2d(2.0, 1.5, Rotation2d.fromDegrees(90))),
      new HolonomicPathFollowingSim.Scenario(
          new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
          new Pose2d(0.3, 0.3, Rotation2d.fromDegrees(150))),
    };

    // Punto inicial: las ganancias que ya estan hardcodeadas ahora mismo en
    // CommandSwerveDrivetrain.configurePathPlanner() (5.0, 0, 0 para
    // traslacion y rotacion).
    double[] initialGuess = {5.0, 0.0, 5.0, 0.0};
    double[] initialStep = {2.0, 0.5, 2.0, 0.5};

    NelderMeadOptimizer.Result result =
        NelderMeadOptimizer.minimize(
            gains -> HolonomicPathFollowingSim.totalCost(gains, scenarios),
            initialGuess,
            initialStep,
            400,
            1e-6);

    double kPTrans = Math.max(0, result.point[0]);
    double kDTrans = Math.max(0, result.point[1]);
    double kPRot = Math.max(0, result.point[2]);
    double kDRot = Math.max(0, result.point[3]);

    System.out.println("===== PID Autotune (simulacion, Nelder-Mead) =====");
    System.out.printf("Costo final: %.6f%n", result.cost);
    System.out.printf("Traslacion -> kP=%.3f kI=0.0 kD=%.3f%n", kPTrans, kDTrans);
    System.out.printf("Rotacion   -> kP=%.3f kI=0.0 kD=%.3f%n", kPRot, kDRot);
    System.out.println();
    System.out.println("Pega esto en CommandSwerveDrivetrain.configurePathPlanner():");
    System.out.printf(
        "new PPHolonomicDriveController(%n"
            + "    new PIDConstants(%.3f, 0.0, %.3f), // PID de Traslacion%n"
            + "    new PIDConstants(%.3f, 0.0, %.3f)  // PID de Rotacion%n"
            + "),%n",
        kPTrans, kDTrans, kPRot, kDRot);

    // Sanity check nada mas -- esto no reemplaza probar en el robot real.
    assertFalse(Double.isNaN(result.cost), "El costo del autotune salio NaN, revisa el modelo");
  }
}
