package frc.robot.tuning;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * Modelo ligero (sin HAL, sin hardware, sin paths de PathPlanner) de un
 * chasis swerve siguiendo una trayectoria holonomica.
 *
 * <p>Replica la misma ley de control que usa internamente {@code
 * PPHolonomicDriveController} de PathPlanner: un PID independiente en X y en
 * Y (las ganancias de traslacion) mas un PID de entrada continua en heading
 * (las ganancias de rotacion), sumados a la velocidad de feedforward del
 * perfil de referencia. Por eso las ganancias que salgan de aqui se pegan
 * directo en {@code new PIDConstants(kP, 0, kD)} dentro de {@link
 * frc.robot.modules.swerve.CommandSwerveDrivetrain#configurePathPlanner()}.
 *
 * <p>No depende de {@code RobotConfig.fromGUISettings()} ni de paths
 * exportados de la GUI de PathPlanner a proposito: esto corre como un test
 * de JUnit normal, rapido y determinista, sin tocar el disco ni el HAL.
 */
public final class HolonomicPathFollowingSim {

  // --- Limites del chasis, calcados de CommandSwerveDrivetrain.pathConstraints
  // y SwerveRequestFactory.MaxSpeed. Si cambias esos, cambia tambien esto. ---
  public static final double MAX_VEL_MPS = 4.5;
  public static final double MAX_ACCEL_MPSS = 4.0;
  public static final double MAX_ANGULAR_VEL_RADPS = Math.toRadians(540);
  public static final double MAX_ANGULAR_ACCEL_RADPSS = Math.toRadians(720);

  // --- Modelo de respuesta del chasis, a partir de tus rutinas de SysId ---
  // TODO(tu equipo): reemplaza estos 4 valores con los kS/kV/kA reales que
  // salieron del analisis de SysId (la app SysId de WPILib, sobre los logs de
  // SysIdRoutineManager.m_sysIdRoutineTranslation / m_sysIdRoutineRotation).
  // tau = kA / kV es la constante de tiempo de primer orden del lazo de
  // velocidad ya cerrado (modulo + Slot0 PID); kS se modela como una banda
  // muerta de velocidad equivalente. Con valores mas fieles, las ganancias
  // que te de el autotune se van a parecer mas a lo que necesitas en el
  // robot real.
  public static double translationTauSeconds = 0.08;
  public static double translationKsMetersPerSec = 0.03;
  public static double rotationTauSeconds = 0.05;
  public static double rotationKsRadPerSec = 0.02;

  public static final double DT = 0.02; // 50 Hz, igual que el loop del robot real
  private static final double HORIZON_SECONDS = 5.0;

  private HolonomicPathFollowingSim() {}

  /** Un tramo (pose inicial -> pose final) representativo de un auto de FRC. */
  public static final class Scenario {
    final Pose2d start;
    final Pose2d end;

    public Scenario(Pose2d start, Pose2d end) {
      this.start = start;
      this.end = end;
    }
  }

  /**
   * Corre un escenario completo con las ganancias dadas y regresa un costo
   * escalar: mas bajo es mejor tracking (menos error de posicion/heading,
   * sin ganancias tan agresivas que generen esfuerzo de control absurdo).
   *
   * @param gains [kPTrans, kDTrans, kPRot, kDRot] (kI se deja en 0, como es
   *     usual para PPHolonomicDriveController)
   */
  public static double cost(double[] gains, Scenario scenario) {
    double kPTrans = Math.max(0, gains[0]);
    double kDTrans = Math.max(0, gains[1]);
    double kPRot = Math.max(0, gains[2]);
    double kDRot = Math.max(0, gains[3]);

    PIDController xController = new PIDController(kPTrans, 0, kDTrans, DT);
    PIDController yController = new PIDController(kPTrans, 0, kDTrans, DT);
    PIDController thetaController = new PIDController(kPRot, 0, kDRot, DT);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    double dx = scenario.end.getX() - scenario.start.getX();
    double dy = scenario.end.getY() - scenario.start.getY();
    double distance = Math.hypot(dx, dy);
    double dirX = distance > 1e-6 ? dx / distance : 0;
    double dirY = distance > 1e-6 ? dy / distance : 0;

    TrapezoidProfile linearProfile =
        new TrapezoidProfile(new TrapezoidProfile.Constraints(MAX_VEL_MPS, MAX_ACCEL_MPSS));
    TrapezoidProfile angularProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(MAX_ANGULAR_VEL_RADPS, MAX_ANGULAR_ACCEL_RADPSS));

    double startAngle = scenario.start.getRotation().getRadians();
    double endAngle = scenario.end.getRotation().getRadians();
    double angleDelta = MathUtil.angleModulus(endAngle - startAngle);

    TrapezoidProfile.State linearGoal = new TrapezoidProfile.State(distance, 0);
    TrapezoidProfile.State angularGoal = new TrapezoidProfile.State(angleDelta, 0);
    TrapezoidProfile.State linearRef = new TrapezoidProfile.State(0, 0);
    TrapezoidProfile.State angularRef = new TrapezoidProfile.State(0, 0);

    // Estado real simulado del chasis (con retraso de primer orden respecto
    // al comando, como el chasis real).
    double x = scenario.start.getX();
    double y = scenario.start.getY();
    double theta = startAngle;
    double vx = 0;
    double vy = 0;
    double omega = 0;

    int steps = (int) Math.ceil(HORIZON_SECONDS / DT);

    double positionErrorCost = 0;
    double headingErrorCost = 0;
    double effortCost = 0;

    for (int i = 0; i < steps; i++) {
      // El perfil de referencia avanza de forma independiente del tracking
      // real (igual que un path pre-generado de PathPlanner): cada paso
      // recalcula el trapezoide minimo desde su propio estado hasta la meta.
      linearRef = linearProfile.calculate(DT, linearRef, linearGoal);
      angularRef = angularProfile.calculate(DT, angularRef, angularGoal);

      double refX = scenario.start.getX() + dirX * linearRef.position;
      double refY = scenario.start.getY() + dirY * linearRef.position;
      double refTheta = MathUtil.angleModulus(startAngle + angularRef.position);
      double refVx = dirX * linearRef.velocity;
      double refVy = dirY * linearRef.velocity;
      double refOmega = angularRef.velocity;

      double cmdVx = refVx + xController.calculate(x, refX);
      double cmdVy = refVy + yController.calculate(y, refY);
      double cmdOmega = refOmega + thetaController.calculate(theta, refTheta);

      cmdVx = MathUtil.clamp(cmdVx, -MAX_VEL_MPS, MAX_VEL_MPS);
      cmdVy = MathUtil.clamp(cmdVy, -MAX_VEL_MPS, MAX_VEL_MPS);
      cmdOmega = MathUtil.clamp(cmdOmega, -MAX_ANGULAR_VEL_RADPS, MAX_ANGULAR_VEL_RADPS);

      vx = firstOrderStep(vx, applyDeadband(cmdVx, translationKsMetersPerSec), translationTauSeconds);
      vy = firstOrderStep(vy, applyDeadband(cmdVy, translationKsMetersPerSec), translationTauSeconds);
      omega = firstOrderStep(omega, applyDeadband(cmdOmega, rotationKsRadPerSec), rotationTauSeconds);

      x += vx * DT;
      y += vy * DT;
      theta = MathUtil.angleModulus(theta + omega * DT);

      double posErr = Math.hypot(refX - x, refY - y);
      double headingErr = Math.abs(MathUtil.angleModulus(refTheta - theta));

      positionErrorCost += posErr * posErr;
      headingErrorCost += headingErr * headingErr;
      effortCost += (cmdVx * cmdVx + cmdVy * cmdVy) * 1e-4 + (cmdOmega * cmdOmega) * 1e-5;
    }

    double finalPosError = Math.hypot(scenario.end.getX() - x, scenario.end.getY() - y);
    double finalHeadingError =
        Math.abs(MathUtil.angleModulus(scenario.end.getRotation().getRadians() - theta));

    return positionErrorCost * DT
        + headingErrorCost * DT * 0.5
        + effortCost * DT
        + finalPosError * finalPosError * 20.0
        + finalHeadingError * finalHeadingError * 10.0;
  }

  /** Corre varios escenarios representativos y suma el costo, para no sobreajustar a uno solo. */
  public static double totalCost(double[] gains, Scenario[] scenarios) {
    double total = 0;
    for (Scenario s : scenarios) {
      total += cost(gains, s);
    }
    return total;
  }

  private static double firstOrderStep(double current, double target, double tau) {
    if (tau <= 1e-6) {
      return target;
    }
    double alpha = DT / (tau + DT);
    return current + alpha * (target - current);
  }

  private static double applyDeadband(double value, double deadband) {
    if (Math.abs(value) < deadband) {
      return 0;
    }
    return value - Math.copySign(deadband, value);
  }
}
