#!/usr/bin/env python3.10
# -*- coding: UTF-8 -*-

from localsolver import (LocalSolver, LSModel, LSExpression)
from typing import (Iterator, Optional, cast)
from networkx import Graph

# References
# https://www.localsolver.com/docs/last/exampletour/vrp.html
# https://www.localsolver.com/docs/last/pythonapi/localsolver/index.html


def getMininumVehicleRouting(*, allDistances: list[list[float]],
                             baseDistance: list[float],
                             minimumDominatingSet: set[int], timeLimit: float,
                             quiet: bool = True) -> float:

  print(f"Running VRP solver. Minimum dominating set size: {len(minimumDominatingSet)}.")

  # Total number of vertices (costumers or antennas) to visit (all in the MDS)
  nodeNumber: int = len(minimumDominatingSet)

  # Exceptionally, the total vehicle number is equal to the number of vertices in the MDS
  vehicleNumber: int = nodeNumber

  # All vehicles have the same autonomy (travelling capacity)
  autonomy: float = 2 * max(baseDistance[i] for i in minimumDominatingSet)

  with LocalSolver() as localSolver:
    # Declares the optimization model
    model: LSModel = localSolver.model

    # Create LocalSolver arrays to be able to access them with an "at" operator
    distanceMatrix = model.array()
    for i in range(len(allDistances)):
      if i in minimumDominatingSet:
        distanceMatrix.add_operand(
          model.array(allDistances[i][j]
                      for j in range(len(allDistances))
                      if j in minimumDominatingSet))

    # The sequence of customers visited by each vehicle
    nodeSequence: list[LSExpression] = [model.list(nodeNumber)] * vehicleNumber

    # Debug only
    if not quiet:

      # A vehicle is used if it visits at least one customer
      vehiclesUsed: list[LSExpression] = [(model.count(nodeSequence[k]) > 0)
                                          for k in range(vehicleNumber)]

      vehiclesUsedNumber: LSExpression = model.sum(vehiclesUsed)

    # All customers must be visited by the vehicles
    model.constraint(model.partition(nodeSequence))

    routeDistanceList: list[Optional[LSExpression]] = [None] * vehicleNumber

    baseDistanceArray: LSExpression = model.array(
      baseDistance[i]
      for i in range(len(baseDistance))
      if i in minimumDominatingSet)

    for k in range(vehicleNumber):
      customerSequence: LSExpression = nodeSequence[k]
      costumerCount: LSExpression = model.count(customerSequence)

      # Distance traveled by each vehicle
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

      model.constraint(routeDistanceList[k] <= autonomy)

    # Total distance traveled
    totalDistance: LSExpression = model.sum(routeDistanceList)

    # Objective: minimize the distance traveled
    # model.minimize(vehiclesUsedNumber) # It's not necessary to minimize the number of refills/vehicles
    model.minimize(totalDistance)

    model.close()

    # Parameterizes the solver
    localSolver.param.time_limit = timeLimit

    # Effectivelly executes the solver
    localSolver.solve()

    # Debug only
    print(f"Total distance travelled: {totalDistance.value}")

    if not quiet:
      print("VRP Solution (0 is the base):")

      # Generate a translation dictionary from the solver indexing to the actual IDs
      auxIter: Iterator = iter(range(len(allDistances)))
      indexToIdDict: dict[int, int] = {
        next(auxIter): i + 1
        for i in range(len(allDistances))
        if i in minimumDominatingSet
      }
      indexToIdDict[None] = 0  # Base

      for k in range(vehicleNumber):
        if not cast(bool, vehiclesUsed[k].value):
          continue

        path: Graph = Graph()
        prevCustomer: Optional[int] = None
        pathCost: float = 0

        for customer in cast(list[int], nodeSequence[k].value):
          distance: float = (baseDistanceArray[customer] if prevCustomer == None
                             else distanceMatrix[prevCustomer][customer])

          path.add_edge((
            indexToIdDict[prevCustomer],
            indexToIdDict[customer],
            {
              "distance": distance
            },
          ))
          prevCustomer = customer
          pathCost += distance

        distance: float = baseDistanceArray[prevCustomer]
        path.add_edge((
          indexToIdDict[prevCustomer],
          indexToIdDict[None],
          {
            "distance": distance
          },
        ))
        pathCost += distance

        print(f"The {k}-th vehicle/refill path is used costing {pathCost}: {path}")

    return cast(float, totalDistance.value)
