#!/usr/bin/env python3.10
# -*- coding: UTF-8 -*-

from localsolver import (LocalSolver, LSModel, LSExpression, LSSolutionStatus)
from typing import (Iterator, Optional, cast)
from networkx import MultiDiGraph
import networkx

# References
# https://www.localsolver.com/docs/last/exampletour/vrp.html
# https://www.localsolver.com/docs/last/pythonapi/localsolver/index.html

INFINITY: float = float("inf")


def getMininumVehicleRouting(*, seed: int, allDistances: list[list[float]],
                             baseDistance: list[float],
                             minimumDominatingSet: set[int], timeLimit: float,
                             quiet: bool = True) -> float:
  # Total number of vertices (costumers or antennas) to visit (all in the MDS)
  nodeNumber: int = len(minimumDominatingSet)

  # Exceptionally, the total vehicle number is equal to the number of vertices in the MDS
  vehicleNumber: int = nodeNumber

  # All vehicles have the same autonomy (travelling capacity)
  autonomy: float = 2 * max(baseDistance[i] for i in minimumDominatingSet)

  with LocalSolver() as localSolver:
    # Declares the optimization model
    model: LSModel = localSolver.model

    # Set the local solver random seed
    localSolver.get_param().set_seed(seed)

    # Create LocalSolver arrays to be able to access them with an "at" operator
    distanceMatrix = model.array()
    for i in range(len(allDistances)):
      if i in minimumDominatingSet:
        distanceMatrix.add_operand(
          model.array(allDistances[i][j]
                      for j in range(len(allDistances))
                      if j in minimumDominatingSet))

    # The distance for each customer from the base
    baseDistanceArray: LSExpression = model.array(
      baseDistance[i]
      for i in range(len(baseDistance))
      if i in minimumDominatingSet)

    # The sequence of customers visited by each vehicle
    nodeSequence: list[LSExpression] = [
      model.list(nodeNumber) for _ in range(vehicleNumber)
    ]

    # Debug only
    if not quiet:

      # A vehicle is used if it visits at least one customer
      vehiclesUsed: list[LSExpression] = [(model.count(nodeSequence[k]) > 0)
                                          for k in range(vehicleNumber)]

      vehiclesUsedNumber: LSExpression = model.sum(vehiclesUsed)

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
      model.constraint(routeDistanceList[k] <= autonomy)

      del distanceSelectorLambda
      del costumerCount
      del customerSequence
      del k

    # Overall distance traveled
    totalDistance: LSExpression = model.sum(routeDistanceList)

    # Objective: minimize the distance traveled
    # model.minimize(vehiclesUsedNumber) # It's not necessary to minimize the number of refills/vehicles
    model.minimize(totalDistance)
    model.close()

    # Parameterizes the solver
    localSolver.get_param().set_time_limit(timeLimit)
    localSolver.get_param().set_iteration_limit(8000)
    localSolver.get_param().set_verbosity(0 if quiet else 2)

    # Effectivelly executes the solver
    localSolver.solve()

    # Get the execution result
    vrpStatus: LSSolutionStatus = localSolver.get_solution().get_status()
    vrpSolution: float = localSolver.get_solution().get_value(totalDistance)

    # Debug only
    if not quiet:
      print(f"\nTotal distance travelled: {vrpSolution}")
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
        if not cast(bool, vehiclesUsed[k].get_value()):
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
        assert abs(routeDistanceList[k].get_value() - pathCost) < 0.000_001

        print(f"\nThe {k+1}-th vehicle/refill path is used costing {pathCost}!")
        print(f"MultiDiGraph: {networkx.to_edgelist(path)}")

        solutionCost += pathCost

      assert abs(vrpSolution - solutionCost) < 0.000_001
      assert totalDistance.get_value() == vrpSolution

    return (vrpSolution if vrpStatus == LSSolutionStatus.FEASIBLE or
            vrpStatus == LSSolutionStatus.OPTIMAL else INFINITY)
