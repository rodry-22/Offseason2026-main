package frc.robot.tuning;

import java.util.Arrays;
import java.util.function.Function;

/**
 * Nelder-Mead (downhill simplex) optimizer, sin dependencias externas.
 *
 * <p>Se usa para buscar las ganancias de PID que minimizan el costo de
 * seguimiento de trayectoria definido en {@link HolonomicPathFollowingSim},
 * sin necesitar el robot real, el HAL de WPILib, ni gradiente analitico del
 * modelo (el costo no es diferenciable de forma limpia por los clamps y la
 * banda muerta, asi que un optimizador libre de gradiente es la opcion
 * practica aqui).
 */
public final class NelderMeadOptimizer {

  /** Mejor punto encontrado y su costo asociado. */
  public static final class Result {
    public final double[] point;
    public final double cost;

    public Result(double[] point, double cost) {
      this.point = point;
      this.cost = cost;
    }
  }

  private NelderMeadOptimizer() {}

  /**
   * @param costFn funcion a minimizar: recibe un punto n-dimensional, regresa un costo escalar
   * @param initialGuess punto inicial (n-dimensional)
   * @param initialStep que tanto perturbar cada dimension para construir el simplex inicial
   * @param maxIterations limite duro de iteraciones
   * @param tolerance se detiene antes si el rango de costos del simplex cae debajo de esto
   */
  public static Result minimize(
      Function<double[], Double> costFn,
      double[] initialGuess,
      double[] initialStep,
      int maxIterations,
      double tolerance) {

    int n = initialGuess.length;
    double[][] simplex = new double[n + 1][];
    double[] costs = new double[n + 1];

    simplex[0] = initialGuess.clone();
    for (int i = 0; i < n; i++) {
      double[] p = initialGuess.clone();
      p[i] += initialStep[i];
      simplex[i + 1] = p;
    }
    for (int i = 0; i <= n; i++) {
      costs[i] = costFn.apply(simplex[i]);
    }

    final double alpha = 1.0; // reflexion
    final double gamma = 2.0; // expansion
    final double rho = 0.5; // contraccion
    final double sigma = 0.5; // encogimiento

    for (int iter = 0; iter < maxIterations; iter++) {
      Integer[] order = sortedIndices(costs);
      double[][] sortedSimplex = new double[n + 1][];
      double[] sortedCosts = new double[n + 1];
      for (int i = 0; i <= n; i++) {
        sortedSimplex[i] = simplex[order[i]];
        sortedCosts[i] = costs[order[i]];
      }
      simplex = sortedSimplex;
      costs = sortedCosts;

      if (Math.abs(costs[n] - costs[0]) < tolerance) {
        break;
      }

      double[] centroid = new double[n];
      for (int i = 0; i < n; i++) {
        for (int d = 0; d < n; d++) {
          centroid[d] += simplex[i][d];
        }
      }
      for (int d = 0; d < n; d++) {
        centroid[d] /= n;
      }

      double[] worst = simplex[n];
      double[] reflected = new double[n];
      for (int d = 0; d < n; d++) {
        reflected[d] = centroid[d] + alpha * (centroid[d] - worst[d]);
      }
      double reflectedCost = costFn.apply(reflected);

      if (reflectedCost < costs[0]) {
        double[] expanded = new double[n];
        for (int d = 0; d < n; d++) {
          expanded[d] = centroid[d] + gamma * (reflected[d] - centroid[d]);
        }
        double expandedCost = costFn.apply(expanded);
        if (expandedCost < reflectedCost) {
          simplex[n] = expanded;
          costs[n] = expandedCost;
        } else {
          simplex[n] = reflected;
          costs[n] = reflectedCost;
        }
      } else if (reflectedCost < costs[n - 1]) {
        simplex[n] = reflected;
        costs[n] = reflectedCost;
      } else {
        double[] contracted = new double[n];
        for (int d = 0; d < n; d++) {
          contracted[d] = centroid[d] + rho * (worst[d] - centroid[d]);
        }
        double contractedCost = costFn.apply(contracted);
        if (contractedCost < costs[n]) {
          simplex[n] = contracted;
          costs[n] = contractedCost;
        } else {
          for (int i = 1; i <= n; i++) {
            for (int d = 0; d < n; d++) {
              simplex[i][d] = simplex[0][d] + sigma * (simplex[i][d] - simplex[0][d]);
            }
            costs[i] = costFn.apply(simplex[i]);
          }
        }
      }
    }

    int best = sortedIndices(costs)[0];
    return new Result(simplex[best], costs[best]);
  }

  private static Integer[] sortedIndices(double[] values) {
    Integer[] idx = new Integer[values.length];
    for (int i = 0; i < idx.length; i++) {
      idx[i] = i;
    }
    Arrays.sort(idx, (a, b) -> Double.compare(values[a], values[b]));
    return idx;
  }
}
