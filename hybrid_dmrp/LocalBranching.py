#!/usr/bin/env python3.10
# -*- coding: UTF-8 -*-

from docplex.mp.model import (
  Context,
  BinaryVarType,
  ContinuousVarType,
  Model,
  LinearConstraint,
)

from .CplexStatusCodeEnum import (CplexStatusCodeEnum, convertStatus)
from .MinimumDominatingSet import HybridBrkgaSolution
from typing import (Any, Generator, Iterable, Optional)
from itertools import (chain, combinations, product)
from docplex.mp.solution import SolveSolution
from docplex.mp.sdetails import SolveDetails
from docplex.mp.engine import JobSolveStatus
from docplex.mp.linear import LinearExpr
from networkx import MultiDiGraph
import networkx

INFINITY: float = float("inf")


class LocalBranchingSolution(HybridBrkgaSolution):
  """Class the holds the Branch and Bound result."""

  def _adjustCost(cost: float, status: JobSolveStatus) -> float:
    return (cost if status in {
      JobSolveStatus.FEASIBLE_SOLUTION,
      JobSolveStatus.OPTIMAL_SOLUTION,
      JobSolveStatus.UNBOUNDED_SOLUTION,
    } else INFINITY)

  def __init__(self, babCost: float,
               babstatus: JobSolveStatus = JobSolveStatus.INFEASIBLE_SOLUTION,
               babstatusCode: CplexStatusCodeEnum = CplexStatusCodeEnum.CPX_STAT_INFEASIBLE,
               babGraph: MultiDiGraph = MultiDiGraph(),
               **kwargs: dict[str, Any]) -> None:

    # Build the parent object
    super().__init__(**kwargs)

    self.babCost: float = LocalBranchingSolution._adjustCost(babCost, babstatus)
    """The Branch and Bound solution total cost."""

    self.babstatus: JobSolveStatus = babstatus
    """The solver returned status for the Branch and Bound solution."""

    self.babstatusCode: CplexStatusCodeEnum = babstatusCode
    """The solver returned status code for the Branch and Bound solution."""

    self.babGraph: MultiDiGraph = babGraph
    """A graph (paths) the vehicles must travel in this solution."""


def kara2011_F2(solution: HybridBrkgaSolution, *, seed: int,
                allDistances: list[list[float]], baseDistance: list[float],
                isCLI: bool = False,
                quiet: bool = True) -> LocalBranchingSolution:
  """
    This function implements the DVRP arc based integer linear programming
    formulation F2 described in [Imdat KARA, 2011]:

    ```txt
    I. Kara, "Arc based integer programming formulations for the Distance
    Constrained Vehicle Routing problem," 3rd IEEE International Symposium
    on Logistics and Industrial Informatics, 2011, pp. 33-38, doi: 10.1109/LINDI.2011.6031159.
    ```

    This formulation has an order of O(n^2) variables and also an order of O(n^2) constraints.
  """

  # List of node IDs without the base. IDs are [1,N]
  N: list[int] = list(map(lambda i: i + 1, solution.minimumDominatingSet))
  N.sort()

  # List of node IDs including the base. IDs are [0,N]
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

  def A():
    """Original kara2011 A edge set."""
    return ((i, j) for i in V for j in V if i != j)

  with Model(name="MDRP-LA", checker="full") as model:

    ################################
    #### I. KARA - F2 ILP Model ####
    ################################

    # Constraint 7 - x_{ij} must be binary variables - O(n^2)
    X: dict[tuple[int, int], BinaryVarType] = model.binary_var_dict(
      A(),
      name="x",
    )

    # It's not possible to modify the variables!
    """
    for j in V:
      for i in V:
        if i != j:
          model.set_va
          X[i, j] = 0

    for path in solution.pathList:
      for (i, j) in path.edges(data=False):
        X[i, j] = 1
    """

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

    # Constraint 12 - total distance from the base to node j when X[i,j] is True - O(n^2)
    model.add_constraints_(
      model.sum_vars(Y[i, j] for j in V if j != i) -
      model.sum_vars(Y[j, i] for j in V if j != i) -
      model.sum(d(i, j) * X[i, j] for j in V if j != i) == 0 for i in N)

    # Skip constraint 13 - not used in kara2011 F2

    # Constraint 14 - Y[0,i] must be equal to the distance between i and the base when X[i,j] is True, and 0 otherwise - O(n)
    model.add_constraints_(Y[0, i] == d(0, i) * X[0, i] for i in N)

    # Skip constraint 15 - not used in kara2011 F2

    # Constraint 16 - Y[i,j] is 0 when X[i,j] is False, otherwise it gains a smaller upper bound - O(n^2)
    model.add_constraints_(Y[i, j] <= (D - d(j, 0)) * X[i, j]
                           for (i, j) in A()
                           if j != 0)

    # Constraint 17 - Y[i,0] is 0 when X[i,0] is False, otherwise it keeps the normal upper bound - O(n)
    model.add_constraints_(Y[i, 0] <= D * X[i, 0] for i in N)

    # Constraint 18 - Y[i,j] >= d(0,i) +d(i,j) when X[i,j] is True, otherwise it keeps the normal lower bound - O(n^2)
    model.add_constraints_(Y[i, j] >= (d(0, i) + d(i, j)) * X[i, j]
                           for (i, j) in A()
                           if i != 0)

    ################################
    ###### Solver Parameters  ######
    ################################

    # Parameterizes the solver
    context: Context = model.context
    context.cplex_parameters.threads = 0

    # Effectivelly executes the solver
    cplexSolution: Optional[SolveSolution] = model.solve(
      clean_before_solve=True,
      log_output=not quiet,
    )

    ################################
    ########## Model end  ##########
    ################################

    solveDetails: SolveDetails = model.solve_details
    babstatusCode: CplexStatusCodeEnum = CplexStatusCodeEnum(solveDetails.status_code)
    graph: MultiDiGraph = MultiDiGraph()

    # Get the execution result
    if cplexSolution != None:
      babCost: float = model.objective_value
      babstatus: JobSolveStatus = cplexSolution.solve_status

    else:
      babstatus: JobSolveStatus = convertStatus(babstatusCode)
      babCost: float = 0

    # Debug or evaluation
    if not quiet or not isCLI:

      if not quiet:
        # if cplexSolution != None:
        #   print()
        #   cplexSolution.print_mst()

        print(f"\nTotal distance travelled: {babCost}")
        print(f"Solver status code: {babstatusCode.name}")
        print(f"Solver status: {babstatus.name}")
        print("BaB Solution (0 is the base):")

      # Account the solution total cost
      solutionCost: float = 0

      if cplexSolution:
        for (i, j) in A():
          if X[i, j].solution_value == 0:
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
    babstatus,
    babstatusCode,
    babGraph=graph,
    **vars(solution),
  )


def solveLocalBranching(hybridBrkgaSolution: HybridBrkgaSolution, *, seed: int,
                        allDistances: list[list[float]],
                        baseDistance: list[float], isCLI: bool = False,
                        quiet: bool = True) -> LocalBranchingSolution:

  return kara2011_F2(
    hybridBrkgaSolution,
    seed=seed,
    allDistances=allDistances,
    baseDistance=baseDistance,
    isCLI=isCLI,
    quiet=quiet,
  )
