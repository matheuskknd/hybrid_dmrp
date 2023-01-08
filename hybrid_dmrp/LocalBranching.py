#!/usr/bin/env python3.10
# -*- coding: UTF-8 -*-

from docplex.mp.model import (
  Context,
  BinaryVarType,
  ContinuousVarType,
  Model,
)

from .CplexStatusCodeEnum import (CplexStatusCodeEnum, convertStatus)
from .MinimumDominatingSet import HybridBrkgaSolution
from docplex.mp.solution import SolveSolution
from typing import (Any, Generator, Optional)
from docplex.mp.engine import JobSolveStatus
from networkx import MultiDiGraph
import networkx
import timeit
import math

INFINITY: float = float("inf")

# Depends on the running machine
TICKS_PER_SECOND: int = 500


class LocalBranchingSolution(HybridBrkgaSolution):
  """Class the holds the Branch and Bound result."""

  def _adjustCost(cost: float, status: JobSolveStatus) -> float:
    return (cost if status in {
      JobSolveStatus.FEASIBLE_SOLUTION,
      JobSolveStatus.OPTIMAL_SOLUTION,
      JobSolveStatus.UNBOUNDED_SOLUTION,
    } else INFINITY)

  def __init__(self, babCost: float = 0,
               babStatus: JobSolveStatus = JobSolveStatus.UNKNOWN,
               babStatusCode: CplexStatusCodeEnum = CplexStatusCodeEnum.CPX_STAT_ABORT_USER,
               babGraph: MultiDiGraph = MultiDiGraph(),
               babElapsedSeconds: float = 0.0, **kwargs: dict[str,
                                                              Any]) -> None:

    # Build the parent object
    super().__init__(**kwargs)

    self.babCost: float = LocalBranchingSolution._adjustCost(babCost, babStatus)
    """The Branch and Bound solution total cost."""

    self.babStatus: JobSolveStatus = babStatus
    """The solver returned status for the Branch and Bound solution."""

    self.babStatusCode: CplexStatusCodeEnum = babStatusCode
    """The solver returned status code for the Branch and Bound solution."""

    self.babGraph: MultiDiGraph = babGraph
    """A graph (paths) the vehicles must travel in this solution."""

    self.babElapsedSeconds: float = babElapsedSeconds
    """The time spent running the Branch and Bound algorith (seconds)."""


def reduced_bounded_kara2011_F2() -> None:
  """
    This function implements a modified version of the DVRP arc based integer linear\
    programming formulation F2 described in [Imdat KARA, 2011], by: adding a new\
    Local Branching constraint based on [Souto et al., 2021], replacing\
    the directed edge set A by the undirected edge set E [Laporte, 1992] and\
    finally by eliminating from E the edges that can never be traversed\
    [Desrochers and Laporte, 1991].

    This formulation has an order of O(n^2) variables and also an order\
    of O(n^2) constraints.

    ```txt
    I. Kara, "Arc based integer programming formulations for the Distance
    Constrained Vehicle Routing problem", in 3rd IEEE International
    Symposium on Logistics and Industrial Informatics, 2011, pp. 33-38.

    G. Souto, I. Morais, G. R. Mauri, G. M. Ribeiro, and P. H. González, "A
    hybrid matheuristic for the Two-Stage Capacitated Facility Location
    problem", Expert Systems with Applications, vol. 185, p. 115501, 2021.

    G. Laporte, "The vehicle routing problem: An overview of exact and
    approximate algorithms", European Journal of Operational Research, vol.
    59, no. 3, pp. 345-358, 1992.

    M. Desrochers and G. Laporte, "Improvements and extensions to the
    Miller-Tucker-Zemlin subtour elimination constraints", Operations
    Research Letters, vol. 10, no. 1, pp. 27-36, 1991.
    ```
  """
  # TODO


def bounded_kara2011_F2(solution: HybridBrkgaSolution, *, seed: int,
                        allDistances: list[list[float]],
                        baseDistance: list[float], isCLI: bool = False,
                        quiet: bool = True) -> LocalBranchingSolution:
  """
    This function implements a modified version of the DVRP arc based integer linear\
    programming formulation F2 described in [Imdat KARA, 2011], by: adding a new\
    Local Branching constraint based on [Souto et al., 2021].

    This formulation has an order of O(n^2) variables and also an order\
    of O(n^2) constraints.

    ```txt
    I. Kara, "Arc based integer programming formulations for the Distance
    Constrained Vehicle Routing problem", in 3rd IEEE International
    Symposium on Logistics and Industrial Informatics, 2011, pp. 33-38.

    G. Souto, I. Morais, G. R. Mauri, G. M. Ribeiro, and P. H. González, "A
    hybrid matheuristic for the Two-Stage Capacitated Facility Location
    problem", Expert Systems with Applications, vol. 185, p. 115501, 2021.
    ```
  """

  # List of node IDs without the base. IDs are [1,n]
  N: list[int] = list(map(lambda i: i + 1, solution.minimumDominatingSet))
  N.sort()

  # List of node IDs including the base. IDs are [0,n]
  V: list[int] = [0] + N

  # All vehicles have the same autonomy (travelling capacity)
  D: float = solution.autonomy

  def d(i: int, j: int) -> float:
    """Distance between nodes (i,j) in V^2; i != j."""
    if i == 0:
      return baseDistance[j - 1]
    elif j == 0:
      return baseDistance[i - 1]
    else:
      return allDistances[i - 1][j - 1]

  def A() -> Generator[tuple[int, int], None, None]:
    """Original kara2011 A edge set."""
    return ((i, j) for i in V for j in V if i != j)

  # DELTA: int = math.ceil(sum(1 for _ in A()) / 2)
  DELTA: int = math.ceil(sum(1 for _ in A()) / 3)
  # DELTA: int = math.ceil(sum(1 for _ in A()) / 4)
  # DELTA: int = math.ceil(sum(1 for _ in A()) / 5)

  with Model(name="MDRP-LA", checker="full") as model:

    ################################
    #### I. KARA - F2 ILP Model ####
    ################################

    # Constraint 7 - x_{ij} must be binary variables - O(n^2)
    X: dict[tuple[int, int], BinaryVarType] = model.binary_var_dict(
      A(),
      name="x",
    )

    # Objective 1 - minimize the total traveled distance - O(n^2)
    model.minimize(model.sum(d(i, j) * X[i, j] for (i, j) in A()))

    # Constraint 4 - each node j has one incoming edge - O(n^2)
    model.add_constraints_(
      model.sum_vars(X[i, j] for i in V if i != j) == 1 for j in N)

    # Constraint 5 - each node i has one outgoing edge - O(n^2)
    model.add_constraints_(
      model.sum_vars(X[i, j] for j in V if j != i) == 1 for i in N)

    # Skip constraint 6 - not used in kara2011 F2
    # Skip constraint 7 - already done

    # For the next constraints we need continuous variables y_{ij} - O(n^2)
    Y: dict[tuple[int, int], ContinuousVarType] = model.continuous_var_dict(
      A(),
      ub=D,
      lb=0,
      name="y",
    )

    # Skip constraint 8 - not used in kara2011 F2
    # Skip constraint 9 - not used in kara2011 F2
    # Skip constraint 10 - not used in kara2011 F2
    # Skip constraint 11 - not used in kara2011 F2

    # Constraint 12 - total distance from the base to node j when X[i, j] is True - O(n^2)
    model.add_constraints_(
      model.sum_vars(Y[i, j] for j in V if j != i) -
      model.sum_vars(Y[j, i] for j in V if j != i) -
      model.sum(d(i, j) * X[i, j] for j in V if j != i) == 0 for i in N)

    # Skip constraint 13 - not used in kara2011 F2

    # Constraint 14 - Y[0, i] must be equal to the distance between i and the base when X[i, j] is True, and 0 otherwise - O(n)
    model.add_constraints_(Y[0, i] == d(0, i) * X[0, i] for i in N)

    # Skip constraint 15 - not used in kara2011 F2

    # Constraint 16 - X[i, j] is 0 when X[i, j] is False, otherwise it keeps the smaller upper bounds - O(n^2)
    model.add_constraints_(Y[i, j] <= (D - d(j, 0)) * X[i, j]
                           for (i, j) in A()
                           if j != 0)

    # Set strengthened upper bounds
    # model.change_var_upper_bounds((Y[i, j] for (i, j) in A() if j != 0),
    #                               (D - d(j, 0) for (_, j) in A() if j != 0))

    # Set the value to zero when inactive, otherwise keeps the bounds [?, *strengthened]
    # model.add_indicator_constraints_(
    #   model.indicator_constraint(X[i, j], Y[i, j] == 0.0, active_value=0)
    #   for (i, j) in A()
    #   if j != 0)

    # Constraint 17 - Y[i,0] is 0 when X[i, 0] is False, otherwise it keeps the default upper bound - O(n)
    model.add_constraints_(Y[i, 0] <= D * X[i, 0] for i in N)

    # Set the value to zero when inactive, otherwise keeps the bounds [?, D]
    # model.add_indicator_constraints_(
    #   model.indicator_constraint(X[i, 0], Y[i, 0] == 0.0, active_value=0)
    #   for i in N)

    # Constraint 18 - X[i, j] >= d(0,i) +d(i,j) when X[i, j] is True, otherwise it keeps the default lower bound - O(n^2)
    model.add_constraints_(Y[i, j] >= (d(0, i) + d(i, j)) * X[i, j]
                           for (i, j) in A()
                           if i != 0)

    # Set strengthened lower bounds when active, otherwise keeps the bounds [0, ?]
    # model.add_indicator_constraints_(
    #   model.indicator_constraint(X[i, j], Y[i, j] >= d(0, i) + d(i, j))
    #   for (i, j) in A()
    #   if i != 0)

    ################################
    ## Souto et al. based constr. ##
    ################################

    # Constraint 19 (new) - the distance/difference in used edges in both solutions can't be more than DELTA - O(n^2)
    model.add_constraint_(
      model.sum_vars(X[i, j] for (i, j) in A() if not solution.usedInVrp(i, j)) +
      sum(1 for (i, j) in A() if solution.usedInVrp(i, j)) -
      model.sum_vars(X[i, j] for (i, j) in A() if solution.usedInVrp(i, j)) <= DELTA)

    ################################
    ###### Solver Parameters  ######
    ################################

    # Remaining time in seconds: up to 20 minutes
    timelimit: int = math.ceil(20*60 - solution.gaElapsedSeconds)
    # dettimelimit: int = math.ceil(0.99 * TICKS_PER_SECOND * timelimit)

    # https://ibmdecisionoptimization.github.io/docplex-doc/cp/docplex.cp.parameters.py.html
    # It seems to be not the correct parameterization: https://stackoverflow.com/q/69464336
    # import docplex.cp.parameters

    # https://www.ibm.com/docs/en/SSSA5P_12.8.0/ilog.odms.studio.help/pdf/paramcplex.pdf
    context: Context = model.context
    context.cplex_parameters.randomseed = seed  # CPXPARAM_RandomSeed
    context.cplex_parameters.timelimit = timelimit  # CPXPARAM_TimeLimit
    # context.cplex_parameters.dettimelimit = dettimelimit  # CPXPARAM_DetTimeLimit
    context.cplex_parameters.read.datacheck = 0  # CPXPARAM_Read_DataCheck: CPX_DATACHECK_OFF
    context.cplex_parameters.threads = 0  # CPXPARAM_Threads: MAX
    context.cplex_parameters.mip.tolerances.uppercutoff = solution.vrpCost  # CPXPARAM_MIP_Tolerances_UpperCutoff

    del context
    del timelimit
    # del dettimelimit

    # Account the time spent running CPLEX
    startTime: float = timeit.default_timer()

    # Effectivelly executes the solver
    cplexSolution: Optional[SolveSolution] = model.solve(
      clean_before_solve=True,
      log_output=not quiet,
    )

    endTime: float = timeit.default_timer()

    ################################
    ########## Model end  ##########
    ################################

    babStatusCode: CplexStatusCodeEnum = CplexStatusCodeEnum(model.solve_details.status_code)
    graph: MultiDiGraph = MultiDiGraph()

    # Get the execution result
    if cplexSolution != None:
      babCost: float = cplexSolution.objective_value
      babStatus: JobSolveStatus = cplexSolution.solve_status

    else:
      babStatus: JobSolveStatus = convertStatus(babStatusCode)
      babCost: float = 0

    # Debug or evaluation
    if not quiet or not isCLI:

      if not quiet:
        # if cplexSolution != None:
        #   print()
        #   cplexSolution.print_mst()

        print(f"\nTotal distance travelled: {babCost}")
        print(f"Solver status code: {babStatusCode.name}")
        print(f"Solver status: {babStatus.name}")
        print("BaB Solution (0 is the base):")

      # Account the solution total cost
      solutionCost: float = 0

      if cplexSolution:
        for (i, j) in A():
          if not cplexSolution.get_value(X[i, j]) > 0.9:
            continue

          # Save the edge on the solution graph representation
          graph.add_edge(i, j, cost=d(i, j))
          solutionCost += d(i, j)

      # We travel the solution to print it
      if not quiet:
        try:
          visited: set[int] = set()
          k: int = 0

          for n in N:
            if n in visited or not graph.has_edge(0, n):
              continue

            pathCost = graph[0][n][0]["cost"]
            path: MultiDiGraph = MultiDiGraph(((0, n, {"cost": pathCost}), ))
            i: int = n

            while i != 0:
              outEdges: list[tuple[int, int,
                                   dict]] = list(graph.out_edges(i, data=True))

              # Save i as visited, and follow the directed graph
              pathCost += outEdges[0][2]["cost"]
              visited.add(i)

              path.add_edges_from(outEdges)
              i = outEdges[0][1]

            k += 1

            print(f"\nThe {k}-th vehicle/refill path is used costing {pathCost}!")
            print(f"Cycles: {list(networkx.simple_cycles(path))}.")
            print(f"MultiDiGraph: {[e for e in path.edges(data=True)]}")

        except BaseException as e:
          print(f"Error trying to print the solution graph:\n\n{e}.")

    assert abs(babCost - solutionCost) < 0.000_001, f"{(babCost, solutionCost)}"
    del solutionCost

  return LocalBranchingSolution(
    babCost,
    babStatus,
    babStatusCode,
    babGraph=graph,
    babElapsedSeconds=endTime - startTime,
    **vars(solution),
  )


def solveLocalBranching(hybridBrkgaSolution: HybridBrkgaSolution, *, seed: int,
                        allDistances: list[list[float]],
                        baseDistance: list[float], isCLI: bool = False,
                        quiet: bool = True) -> LocalBranchingSolution:

  return bounded_kara2011_F2(
    hybridBrkgaSolution,
    seed=seed,
    allDistances=allDistances,
    baseDistance=baseDistance,
    isCLI=isCLI,
    quiet=quiet,
  )
