#!/usr/bin/env python3.10
# -*- coding: UTF-8 -*-

from docplex.mp.constants import WriteLevel
from docplex.mp.model import (
  Context,
  BinaryVarType,
  IntegerVarType,
  ContinuousVarType,
  Model,
)

from .CplexStatusCodeEnum import (CplexStatusCodeEnum, convertStatus)
from .MinimumDominatingSet import HybridBrkgaSolution
from docplex.mp.solution import SolveSolution
from typing import (Any, Generator, Optional)
from docplex.mp.engine import JobSolveStatus
from .PreProcessing import InstanceData
from networkx import MultiDiGraph
from os.path import exists
import itertools
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

    # Remaining time in seconds: up to 20~60 minutes
    if len(allDistances) < 1_000:
      timelimit: int = math.ceil(max(1, 20*60 - solution.gaElapsedSeconds))
      # dettimelimit: int = math.ceil(0.99 * TICKS_PER_SECOND * timelimit)
      workmem: int = 8_000  # CPX_PARAM_WORKMEM: 8 GB
      emphasis_m: int = 0  # CPXPARAM_Emphasis_Memory: CPX_OFF

    else:
      timelimit: int = math.ceil(max(1, 60*60 - solution.gaElapsedSeconds))
      # dettimelimit: int = math.ceil(0.99 * TICKS_PER_SECOND * timelimit)
      workmem: int = 500  # CPX_PARAM_WORKMEM: 500 MB
      emphasis_m: int = 1  # CPXPARAM_Emphasis_Memory: CPX_ON

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

    # https://xavier-nodet.medium.com/cplex-usage-of-ram-when-solving-continuous-models-3e0170c92f16
    # https://xavier-nodet.medium.com/cplex-memory-usage-for-mips-4eb737f89e7a
    context.cplex_parameters.emphasis.memory = emphasis_m  # CPXPARAM_Emphasis_Memory
    context.cplex_parameters.lpmethod = 6  # CPXPARAM_LPMethod: CPX_ALG_CONCURRENT
    context.cplex_parameters.parallel = -1  # CPXPARAM_Parallel: CPX_PARALLEL_OPPORTUNISTIC

    context.cplex_parameters.workmem = workmem  # CPX_PARAM_WORKMEM
    context.cplex_parameters.mip.strategy.file = 3  # CPX_PARAM_NODEFILEIND: Disk+Compressed
    context.cplex_parameters.mip.limits.treememory = 480_000  # CPX_PARAM_TRELIM: 480 GB
    context.cplex_parameters.mip.strategy.variableselect = 3  # CPX_PARAM_VARSEL: CPX_VARSEL_STRONG

    del context
    del timelimit
    # del dettimelimit
    del workmem
    del emphasis_m

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


def bounded_morais2022(instanceData: InstanceData, solution: HybridBrkgaSolution,
                       *, seed: int, isCLI: bool = False,
                       quiet: bool = True) -> LocalBranchingSolution:
  """
    This function implements a modified version of the MDRPwLA integer linear\
    programming formulation described in [I. Morais et al., 2022], by: \
    adding a new Local Branching constraint based on [Souto et al., 2021].

    This formulation has an order of O(n^3) variables and also an order\
    of O(n^3) constraints.

    ```txt
    I. Morais, V. de Almeida Guimarães, E. B. da Silva, and P. H. González,
    "Prescriptive Analytics in Smart Cities: A Combinatorial Approach in
    Rescue Operations", in Smart Cities, 2022, pp. 131-145.
    ```
  """

  # Instance variables aliases
  allDistances: list[list[float]] = instanceData.distance_matrix
  neighborhood: list[list[int]] = instanceData.communication_net
  baseDistance: list[float] = instanceData.distance_from_base

  # List of node IDs without the base. IDs are [1,n]
  V_: list[int] = [i for i in range(1, len(allDistances) + 1)]

  # List of node IDs including the base. IDs are [0,n]
  V: list[int] = [0] + V_

  # List of cycle (vehicles) IDs - constraint 17
  K: list[int] = V[0:-1]

  # All vehicles have the same autonomy (travelling capacity)
  C: float = 2 * max(baseDistance)

  def d(i: int, j: int) -> float:
    """Distance between nodes (i,j) in V^2; i != j."""
    if i == 0:
      return baseDistance[j - 1]
    elif j == 0:
      return baseDistance[i - 1]
    else:
      return allDistances[i - 1][j - 1]

  def A() -> Generator[tuple[int, int], None, None]:
    """Original morais2022 A edge set."""
    return ((i, j) for i in V for j in V if i != j)

  # Branch and bound constraint constant
  # DELTA: int = math.ceil(sum(1 for _ in A()) / 2)
  DELTA: int = math.ceil(sum(1 for _ in A()) / 3)
  # DELTA: int = math.ceil(sum(1 for _ in A()) / 4)
  # DELTA: int = math.ceil(sum(1 for _ in A()) / 5)

  with Model(name="MDRPwLA", checker="full") as model:

    ################################
    ## MORAIS ET AL. - ILP Model  ##
    ################################

    # Constraint 14 - x_{ijk} must be binary variables - O(n^3)
    X: dict[tuple[int, int, int], BinaryVarType] = model.binary_var_dict(
      ((i, j, k) for ((i, j), k) in itertools.product(A(), K)),
      name="x",
    )

    # Constraint 15 - y_{ik} must be binary variables - O(n^2)
    Y: dict[tuple[int, int], BinaryVarType] = model.binary_var_dict(
      ((i, k) for (i, k) in itertools.product(V, K)),
      name="y",
    )

    # Constraint 16 - z_{ijk} must be positive integer variables - O(n^3)
    Z: dict[tuple[int, int, int], IntegerVarType] = model.integer_var_dict(
      ((i, j, k) for ((i, j), k) in itertools.product(A(), K)),
      name="z",
      lb=0,
    )

    # Objective 2 - minimize the total traveled distance - O(n^3)
    model.minimize(
      model.sum(
        d(i, j) * X[i, j, k] for ((i, j), k) in itertools.product(A(), K)))

    # Constraint 3 - neighborhood coverage - O(n^3)
    model.add_constraints_(
      model.sum_vars(Y[j + 1, k]
                     for j in itertools.chain((i - 1, ), neighborhood[i - 1])
                     for k in K) >= 1
      for i in V_)

    # Constraint 4 (fixed) - at most one vehicle can visit non base nodes - O(n^2)
    model.add_constraints_(model.sum_vars(Y[i, k] for k in K) <= 1 for i in V_)

    # Constraint 5 - the vehicle must leave the node it visited - O(n^3)
    model.add_constraints_(
      model.sum_vars(X[i, j, k]
                     for j in V
                     if i != j) == Y[i, k]
      for i in V
      for k in K)

    # Constraint 6 - the vehicle must go to the node it visits - O(n^3)
    model.add_constraints_(
      model.sum_vars(X[j, i, k]
                     for j in V
                     if i != j) == Y[i, k]
      for i in V
      for k in K)

    # Constraint 7 - at least one vehicle must leave the base - O(n)
    model.add_constraint_(model.sum_vars(Y[0, k] for k in K) >= 1)

    # Constraint 8 - flow conservation: the Z variable values increase by 1 for each visited node - O(n^3)
    model.add_constraints_(
      model.sum_vars(Z[i, j, k] for j in V if i != j) == model.sum_vars(
        Z[j, i, k] for j in V if i != j) + Y[i, k] for i in V_ for k in K)

    # Constraint 9 - flow conservation/upper bound: the Z variable values reach at most the number of visited nodes - O(n^3)
    model.add_constraints_(Z[i, j, k] <= model.sum_vars(Y[l, k]
                                                        for l in V)
                           for ((i, j), k) in itertools.product(A(), K))

    # Constraint 10 - flow conservation/lower bound: the Z variable values are 0 if the node is not visited, or |V| otherwise - O(n^3)
    model.add_constraints_(Z[i, j, k] <= X[i, j, k] * len(V)
                           for ((i, j), k) in itertools.product(A(), K))

    # Constraint 11 - flow conservation: the vehicle must leave the base exactly once if used - O(n^2)
    model.add_constraints_(
      model.sum_vars(Z[0, j, k] for j in V_) == Y[0, k] for k in K)

    # Constraint 12 - flow conservation: the flow going to the base is equal to the number of vehicle visited nodes - O(n^2)
    model.add_constraints_(
      model.sum_vars(Z[j, 0, k]
                     for j in V_) == model.sum_vars(Y[l, k]
                                                    for l in V)
      for k in K)

    # Constraint 13 - the path a vehicle travels must respect its autonomy - O(n^3)
    model.add_constraints_(
      model.sum(d(i, j) * X[i, j, k] for (i, j) in A()) <= C for k in K)

    ################################
    ## Souto et al. based constr. ##
    ################################

    # Constraint 18 (new) - the distance/difference in used edges in both solutions can't be more than DELTA - O(n^3)
    model.add_constraint_(
      model.sum_vars(X[i, j, k] for (
        (i, j), k) in itertools.product(A(), K) if not solution.usedInVrp(i, j)) +
      sum(1 for (i, j) in A() if solution.usedInVrp(i, j)) -
      model.sum_vars(X[i, j, k] for (
        (i, j), k) in itertools.product(A(), K) if solution.usedInVrp(i, j)) <= DELTA)

    ################################
    ########## MIP START  ##########
    ################################

    # https://ibmdecisionoptimization.github.io/docplex-doc/mp/docplex.mp.model.html#docplex.mp.model.Model.add_mip_start
    # https://ibmdecisionoptimization.github.io/docplex-doc/mp/docplex.mp.solution.html#docplex.mp.solution.SolveSolution
    def createMipStart() -> SolveSolution:
      babMipStart: SolveSolution = SolveSolution(model, name="babMipStart")
      vrpGraph: MultiDiGraph = solution.vrpGraph
      curK: int = -1

      for i in V_:
        if not vrpGraph.has_edge(0, i):
          continue

        path: MultiDiGraph = MultiDiGraph()
        curZ: int = 1
        curK += 1

        j: int = i
        i = 0

        while True:
          outEdge: tuple[int, int] = list(vrpGraph.out_edges(j, data=False))[0]

          # Update the model
          babMipStart.add_var_value(Y[j, curK], 1)
          babMipStart.add_var_value(X[i, j, curK], 1)
          babMipStart.add_var_value(Z[i, j, curK], curZ)

          # Follow the directed graph
          path.add_edge(i, j)
          (i, j) = outEdge
          curZ += 1

          if i == 0:
            break

        for i in V_:
          if not path.has_node(i):
            babMipStart.add_var_value(Y[i, curK], 0)

        for (i, j) in A():
          if not path.has_edge(i, j):
            babMipStart.add_var_value(X[i, j, curK], 0)
            babMipStart.add_var_value(Z[i, j, curK], 0)

      # The remaining vehicles do nothing
      while curK != K[-1]:
        curK += 1

        for i in V:
          babMipStart.add_var_value(Y[i, curK], 0)

        for (i, j) in A():
          babMipStart.add_var_value(X[i, j, curK], 0)
          babMipStart.add_var_value(Z[i, j, curK], 0)

      return babMipStart

    # Create a MIP start from the BaB original solution
    babMipStart: SolveSolution = createMipStart()
    assert babMipStart.is_feasible_solution(silent=False)
    assert babMipStart.check_as_mip_start(strong_check=True)
    # babMipStart.export_as_mst(path=".", basename="babMipStart",
    #                           write_level=WriteLevel.AllVars)

    babMipStart = model.add_mip_start(
      babMipStart,
      write_level=WriteLevel.DiscreteVars,
      complete_vars=True,
    )

    assert babMipStart != None, "babMipStart convertion failed!"
    assert babMipStart.is_feasible_solution(silent=False)
    assert babMipStart.check_as_mip_start(strong_check=True)
    del babMipStart

    ################################
    ###### Solver Parameters  ######
    ################################

    # Remaining time in seconds: up to 20~60 minutes
    if len(allDistances) < 1_000:
      timelimit: int = math.ceil(max(1, 20*60 - solution.gaElapsedSeconds))
      # dettimelimit: int = math.ceil(0.99 * TICKS_PER_SECOND * timelimit)
      workmem: int = 8_000  # CPX_PARAM_WORKMEM: 8 GB
      # emphasis_m: int = 0  # CPXPARAM_Emphasis_Memory: CPX_OFF

    else:
      timelimit: int = math.ceil(max(1, 60*60 - solution.gaElapsedSeconds))
      # dettimelimit: int = math.ceil(0.99 * TICKS_PER_SECOND * timelimit)
      workmem: int = 500  # CPX_PARAM_WORKMEM: 500 MB
      # emphasis_m: int = 1  # CPXPARAM_Emphasis_Memory: CPX_ON

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

    # https://xavier-nodet.medium.com/cplex-usage-of-ram-when-solving-continuous-models-3e0170c92f16
    # https://xavier-nodet.medium.com/cplex-memory-usage-for-mips-4eb737f89e7a
    context.cplex_parameters.emphasis.memory = 1  # CPXPARAM_Emphasis_Memory: CPX_ON
    context.cplex_parameters.lpmethod = 6  # CPXPARAM_LPMethod: CPX_ALG_CONCURRENT
    context.cplex_parameters.parallel = -1  # CPXPARAM_Parallel: CPX_PARALLEL_OPPORTUNISTIC

    context.cplex_parameters.workmem = workmem  # CPX_PARAM_WORKMEM
    context.cplex_parameters.mip.strategy.file = 3  # CPX_PARAM_NODEFILEIND: Disk+Compressed
    context.cplex_parameters.mip.limits.treememory = 480_000  # CPX_PARAM_TRELIM: 480 GB
    context.cplex_parameters.mip.strategy.variableselect = 3  # CPX_PARAM_VARSEL: CPX_VARSEL_STRONG

    del context
    del timelimit
    # del dettimelimit
    del workmem
    # del emphasis_m

    # Print the Linear Programming model
    # if not exists("MDRPwLA.lp"):
    #   model.cplex.write("MDRPwLA.lp")

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
        for ((i, j), k) in itertools.product(A(), K):
          if not cplexSolution.get_value(X[i, j, k]) > 0.9:
            continue

          # Save the edge on the solution graph representation
          graph.add_edge(i, j, cost=d(i, j))
          solutionCost += d(i, j)

      # We travel the solution to print it
      if not quiet:
        try:
          visited: set[int] = set()
          k: int = 0

          for n in V_:
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


def solveLocalBranching(instanceData: InstanceData,
                        hybridBrkgaSolution: HybridBrkgaSolution, *, seed: int,
                        isCLI: bool = False,
                        quiet: bool = True) -> LocalBranchingSolution:

  return bounded_morais2022(
    instanceData,
    hybridBrkgaSolution,
    seed=seed,
    isCLI=isCLI,
    quiet=quiet,
  )
