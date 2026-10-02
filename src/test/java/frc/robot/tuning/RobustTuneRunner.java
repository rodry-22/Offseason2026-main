package frc.robot.tuning;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import com.stzteam.forgemini.io.NetworkIO;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.configuration.constants.TunerConstants;
import frc.robot.modules.swerve.CommandSwerveDrivetrain;
import frc.robot.modules.swerve.PathFollowingTuner;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIfEnvironmentVariable;

/**
 * Un escenario del autotune robusto: evalua una lista fija de ganancias del PID de trayectoria
 * contra el mundo de Gazebo que este corriendo en lockstep, a uno o varios voltajes de bateria.
 *
 * <p>No es una prueba normal: solo corre si existe ROBUST_SALIDA, asi que {@code ./gradlew build}
 * la salta. La lanza tools/robust_autotune.py, que es quien levanta cada mundo (masa, mu) y junta
 * los resultados. Variables:
 *
 * <ul>
 *   <li>MARS_LOCKSTEP: el bridge en lockstep, p. ej. 127.0.0.1:5811 (obligatoria).
 *   <li>ROBUST_SALIDA: archivo .jsonl donde se AGREGA una linea por resultado.
 *   <li>ROBUST_ESCENARIO: etiqueta del mundo, p. ej. m62_mu0.8.
 *   <li>ROBUST_GANANCIAS: "kPt,kDt,kPr,kDr;kPt,kDt,kPr,kDr;...".
 *   <li>ROBUST_ROTACION: grados que gira el path de prueba (0 = recto).
 *   <li>ROBUST_VOLTAJES: "12.5,10.5", voltaje de bateria de cada pasada.
 * </ul>
 */
@EnabledIfEnvironmentVariable(named = "ROBUST_SALIDA", matches = ".+")
class RobustTuneRunner {

  @Test
  void evaluarEscenario() throws Exception {
    String salida = System.getenv("ROBUST_SALIDA");
    String escenario = System.getenv().getOrDefault("ROBUST_ESCENARIO", "nominal");
    double rotacion = Double.parseDouble(System.getenv().getOrDefault("ROBUST_ROTACION", "0"));
    List<double[]> ganancias = parsearGanancias(System.getenv("ROBUST_GANANCIAS"));
    double[] voltajes = parsearLista(System.getenv().getOrDefault("ROBUST_VOLTAJES", "12.0"));

    assertTrue(HAL.initialize(500, 0));
    NetworkTableInstance.getDefault().startServer();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();

    CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    assertTrue(
        NetworkIO.get("MARS/Glue/diag", "lockstep", false),
        "sin lockstep: hace falta MARS_LOCKSTEP y el bridge con --lockstep-port");
    PathFollowingTuner tuner = new PathFollowingTuner(drivetrain);

    // Lo que haria TimedRobot: el ciclo de 20 ms en tiempo SIMULADO. El comando se programa
    // DENTRO de este hilo: el scheduler no es seguro entre hilos.
    AtomicReference<Command> pendiente = new AtomicReference<>();
    Notifier ciclo =
        new Notifier(
            () -> {
              Unmanaged.feedEnable(1000);
              DriverStationSim.notifyNewData();
              DriverStation.refreshData();
              Command c = pendiente.getAndSet(null);
              if (c != null) {
                CommandScheduler.getInstance().schedule(c);
              }
              CommandScheduler.getInstance().run();
            });
    ciclo.startPeriodic(0.02);
    esperarSimulado(1.0);

    try (PrintWriter out = new PrintWriter(new FileWriter(salida, true))) {
      for (double voltaje : voltajes) {
        RoboRioSim.setVInVoltage(voltaje);
        esperarSimulado(0.2);
        List<PathFollowingTuner.Resultado> resultados = new CopyOnWriteArrayList<>();
        AtomicBoolean terminado = new AtomicBoolean(false);
        pendiente.set(
            tuner.evaluarCommand(ganancias, rotacion, resultados::add)
                .finallyDo(() -> terminado.set(true)));
        while (!terminado.get()) {
          Thread.sleep(20);
        }
        for (var r : resultados) {
          out.println(json(escenario, voltaje, r));
        }
        out.flush();
        System.out.printf(
            Locale.ROOT, "[ROBUST] %s @ %.1f V: %d resultados%n", escenario, voltaje, resultados.size());
      }
    } finally {
      ciclo.stop();
    }
  }

  private static void esperarSimulado(double segundos) throws InterruptedException {
    double t0 = Timer.getFPGATimestamp();
    while (Timer.getFPGATimestamp() - t0 < segundos) {
      Thread.sleep(5);
    }
  }

  private static String json(String escenario, double voltaje, PathFollowingTuner.Resultado r) {
    double[] g = r.ganancias();
    return String.format(
        Locale.ROOT,
        "{\"escenario\":\"%s\",\"voltaje\":%.2f,\"rotacion\":%.1f,\"kPt\":%.4f,\"kDt\":%.4f,"
            + "\"kPr\":%.4f,\"kDr\":%.4f,\"costo\":%.6f,\"rms_cm\":%.3f,\"max_cm\":%.3f,"
            + "\"final_cm\":%.3f,\"rms_head_deg\":%.3f,\"abortado\":%b}",
        escenario, voltaje, r.rotacionDeg(), g[0], g[1], g[2], g[3], r.costo(),
        r.rmsPosM() * 100, r.maxPosM() * 100, r.finalPosM() * 100,
        Math.toDegrees(r.rmsHeadingRad()), r.abortado());
  }

  private static List<double[]> parsearGanancias(String texto) throws IOException {
    if (texto == null || texto.isBlank()) {
      throw new IOException("falta ROBUST_GANANCIAS");
    }
    List<double[]> lista = new ArrayList<>();
    for (String juego : texto.split(";")) {
      double[] g = parsearLista(juego);
      if (g.length != 4) {
        throw new IOException("cada juego de ganancias lleva 4 valores: " + juego);
      }
      lista.add(g);
    }
    return lista;
  }

  private static double[] parsearLista(String texto) {
    String[] partes = texto.trim().split(",");
    double[] v = new double[partes.length];
    for (int i = 0; i < partes.length; i++) {
      v[i] = Double.parseDouble(partes[i].trim());
    }
    return v;
  }
}
