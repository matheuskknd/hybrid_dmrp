#!/usr/bin/env python3.10
# -*- coding: UTF-8 -*-

from localsolver import (
  LocalSolver,
  LSModel,
  LSExpression,
  LSSolutionStatus,
  LSParam,
)

from typing import (Iterator, Optional, cast)
from networkx import MultiDiGraph
from .utils import extended_enum_new
from unittest.mock import patch
import networkx

# References
# https://www.localsolver.com/docs/last/exampletour/vrp.html
# https://www.localsolver.com/docs/last/pythonapi/localsolver/index.html

INFINITY: float = float("inf")


class VRPSolution:
  """Class the holds the VRP-LA subproblem result."""

  def _adjustCost(cost: float, status: LSSolutionStatus) -> float:
    return (cost if status == LSSolutionStatus.FEASIBLE or
            status == LSSolutionStatus.OPTIMAL else INFINITY)

  def __init__(self, vrpCost: float, vrpStatus: LSSolutionStatus | str |
               int = LSSolutionStatus.INFEASIBLE, autonomy: float = 0.0, *,
               vrpGraph: MultiDiGraph = MultiDiGraph()) -> None:

    with patch("localsolver.LSSolutionStatus.__new__",
               new=extended_enum_new(LSSolutionStatus)):

      self.vrpStatus: LSSolutionStatus = LSSolutionStatus(vrpStatus)
      """The solver returned status for the VRP solution."""

    self.vrpCost: float = VRPSolution._adjustCost(vrpCost, self.vrpStatus)
    """The VRP solution total cost."""

    self.autonomy: float = autonomy
    """The vehicle autonomy."""

    self.vrpGraph: MultiDiGraph = vrpGraph
    """A graph (paths) the vehicles must travel in this solution."""

  def usedInVrp(self, i: int, j: int) -> bool:
    """Return True if the edge (i, j) is used by the VRP solution, False otherwise."""
    return self.vrpGraph.has_edge(i, j)


def getMininumVehicleRouting(*, seed: int, allDistances: list[list[float]],
                             baseDistance: list[float],
                             minimumDominatingSet: set[int], timeLimit: float,
                             bestVrpCost: float = INFINITY,
                             quiet: bool = True) -> VRPSolution:

  # The same as the dominating set, but ordered
  sortedDominatingSet: list[int] = sorted(minimumDominatingSet)

  # Total number of vertices (costumers or antennas) to visit (all in the MDS)
  nodeNumber: int = len(minimumDominatingSet)

  # Exceptionally, the total vehicle number is equal to the number of vertices in the MDS
  vehicleNumber: int = nodeNumber

  # All vehicles have the same autonomy (travelling capacity)
  autonomy: float = 2 * max(baseDistance[i] for i in minimumDominatingSet)

  with LocalSolver() as localSolver:
    # Declares the optimization model
    model: LSModel = localSolver.model

    # Create a constant expression to be the autonomy
    autonomyConst: LSExpression = model.create_constant(autonomy)

    # Create LocalSolver arrays to be able to access them with an "at" operator
    distanceMatrix = model.array(
      model.array(allDistances[i][j]
                  for j in sortedDominatingSet)
      for i in sortedDominatingSet)

    # The distance for each customer from the base
    baseDistanceArray: LSExpression = model.array(baseDistance[i]
                                                  for i in sortedDominatingSet)

    # The sequence of customers visited by each vehicle
    nodeSequence: list[LSExpression] = [
      model.list(nodeNumber) for _ in range(vehicleNumber)
    ]

    # A vehicle is used if it visits at least one customer
    # if not quiet:
    #   vehiclesUsed: list[LSExpression] = [(model.count(nodeSequence[k]) > 0)
    #                                     for k in range(vehicleNumber)]

    #   vehiclesUsedNumber: LSExpression = model.sum(vehiclesUsed)

    # No client must be visited more than once by a single vehicle
    # And all customers must be visited by some vehicle
    model.constraint(model.partition(nodeSequence))

    # The total distance travelled by each vehicle
    routeDistanceList: list[Optional[LSExpression]] = [None] * vehicleNumber

    for k in range(vehicleNumber):
      customerSequence: LSExpression = nodeSequence[k]
      costumerCount: LSExpression = model.count(customerSequence)

      # Distance traveled by this vehicle from node i-1 to node i
      def distanceSelectorLambda(i: int) -> LSExpression:
        return model.at(distanceMatrix, customerSequence[i - 1],
                        customerSequence[i])

      routeDistanceList[k] = (model.sum(
        model.range(1, costumerCount),
        model.lambda_function(func=distanceSelectorLambda),
      ) + model.iif(
        costumerCount > 0,
        baseDistanceArray[customerSequence[0]] +
        baseDistanceArray[customerSequence[costumerCount - 1]],
        0,
      ))

      # The autonomy constraint is the same for all vehicles
      model.constraint(routeDistanceList[k] <= autonomyConst)

      del distanceSelectorLambda
      del costumerCount
      del customerSequence
      del k

    # Overall distance traveled
    totalDistance: LSExpression = model.sum(routeDistanceList)

    # Objective: minimize the distance traveled
    # model.minimize(vehiclesUsedNumber) # It's not necessary to minimize the number of refills/vehicles
    model.minimize(totalDistance)

    # Parameterizes the solver
    lsParam: LSParam = localSolver.get_param()
    lsParam.set_iteration_between_ticks(300_000)
    lsParam.set_time_between_displays(300_000)
    lsParam.set_time_between_ticks(300_000)
    lsParam.set_nb_threads(0)

    if quiet:
      lsParam.set_log_writer(None)
      lsParam.set_verbosity(0)
    else:
      lsParam.set_verbosity(2)

    # Set the local solver random seed + iteration_limit to make it deterministic
    #lsParam.set_time_limit(timeLimit) # Non deterministic
    lsParam.set_iteration_limit(8000)
    lsParam.set_seed(seed)

    # Effectivelly executes the solver
    model.close()
    localSolver.solve()

    # Get the execution result
    vrpStatus: LSSolutionStatus = localSolver.get_solution().get_status()
    vrpCost: float = localSolver.get_solution().get_value(totalDistance)
    graph: MultiDiGraph = MultiDiGraph()

    # Debug or evaluation
    if not quiet or vrpCost < bestVrpCost:

      if not quiet:
        print(f"\nTotal distance travelled: {vrpCost}")
        print("VRP Solution (0 is the base):")

      # Recreate the constant variables (accessing them in the model is not allowed by LocalSolver)
      _distanceMatrix: list[list[float]] = []
      for i in range(len(allDistances)):
        if i in minimumDominatingSet:
          _distanceMatrix.append([
            allDistances[i][j]
            for j in range(len(allDistances))
            if j in minimumDominatingSet
          ])

      _baseDistanceArray: list[float] = [
        baseDistance[i]
        for i in range(len(baseDistance))
        if i in minimumDominatingSet
      ]

      # Generate a translation dictionary from the solver indexing to the actual IDs
      auxIter: Iterator = iter(range(len(allDistances)))
      indexToIdDict: dict[Optional[int], int] = {
        next(auxIter): i + 1
        for i in range(len(allDistances))
        if i in minimumDominatingSet
      }
      indexToIdDict[None] = 0  # Base
      solutionCost: float = 0

      for k in range(vehicleNumber):
        if len(nodeSequence[k].get_value()) == 0:
          continue

        path: MultiDiGraph = MultiDiGraph()
        prevCustomer: Optional[int] = None
        pathCost: float = 0

        for customer in cast(list[int], nodeSequence[k].get_value()):
          distance: float = (_baseDistanceArray[customer] if prevCustomer == None
                             else _distanceMatrix[prevCustomer][customer])

          path.add_edge(
            indexToIdDict[prevCustomer],
            indexToIdDict[customer],
            cost=distance,
          )
          prevCustomer = customer
          pathCost += distance

        distance: float = _baseDistanceArray[prevCustomer]
        path.add_edge(
          indexToIdDict[prevCustomer],
          indexToIdDict[None],
          cost=distance,
        )

        pathCost += distance
        rDk: float = routeDistanceList[k].get_value()
        assert abs(rDk - pathCost) < 0.000_001, f"{(rDk, pathCost)}"

        if not quiet:
          print(f"\nThe {k+1}-th vehicle/refill path is used costing {pathCost}!")
          print(f"MultiDiGraph: {[e for e in path.edges(data=True)]}")

        solutionCost += pathCost
        graph = networkx.compose(graph, path)

      assert abs(vrpCost - solutionCost) < 0.000_001, f"{(vrpCost, solutionCost)}"
      assert totalDistance.get_value() == vrpCost

    return VRPSolution(vrpCost, vrpStatus, autonomy, vrpGraph=graph)
