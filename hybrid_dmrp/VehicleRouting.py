#!/usr/bin/env python3.10
# -*- coding: UTF-8 -*-

import sys
from typing import (Optional, Tuple)
from localsolver import (LocalSolver, LSModel, LSExpression)

# References
# https://www.localsolver.com/docs/last/exampletour/vrp.html
# https://www.localsolver.com/docs/last/pythonapi/localsolver/index.html

def convertMinimumDominatingSet(
  *, allDistances: list[list[float]], baseDistance: list[float],
  minimumDominatingSet: set[int]
) -> Tuple[list[list[float]], list[float], dict[int, int]]:

  # Copy the original variables to new simplified ones
  newAllDistances = []
  newBaseDistance = []

  # Save a mapping for the previous indexes
  convertionMap: dict[int, int] = dict()

  for i in range(1, len(baseDistance)+1):
    index = i-1
    if i in minimumDominatingSet:      
      newBaseDistance.append(baseDistance[index])
      newAllDistances.append([])

      convertionMap[len(newAllDistances)] = index

      for j in range(1, len(baseDistance)+1):
        index2 = j-1
        if j in minimumDominatingSet:
          newAllDistances[-1].append(allDistances[index][index2])

  return (newAllDistances, newBaseDistance, convertionMap)


def getMininumVehicleRouting(*, allDistances: list[list[float]],
                             baseDistance: list[float],
                             minimumDominatingSet: set[int],
                             timeLimit: float = float("inf")) -> None:

  # Remove the nodes that doesn't belong to the MDP
  (newAllDistances, newBaseDistance,
   convertionMap) = convertMinimumDominatingSet(
     allDistances=allDistances, baseDistance=baseDistance,
     minimumDominatingSet=minimumDominatingSet)

  # Total number of vertices (costumers or antennas) to visit (all in the MDS)
  nodeNumber: int = len(minimumDominatingSet)

  # Exceptionally, the total vehicle number is equal to the number of vertices in the MDS
  vehicleNumber: int = nodeNumber

  # All vehicles have the same autonomy (travelling capacity)
  autonomy: float = 2 * max(newBaseDistance)

  with LocalSolver() as localSolver:
    # Declares the optimization model
    model: LSModel = localSolver.model

    # Create LocalSolver arrays to be able to access them with an "at" operator
    distanceMatrix = model.array()
    for i in range(nodeNumber):
      distanceMatrix.add_operand(model.array(newAllDistances[i]))

    # The sequence of customers visited by each vehicle
    nodeSequence: list[LSExpression] = [model.list(nodeNumber)] * vehicleNumber

    # All customers must be visited by the vehicles
    model.constraint(model.partition(nodeSequence))

    # A vehicle is used if it visits at least one customer
    vehiclesUsed: list[LSExpression] = [(model.count(nodeSequence[k]) > 0)
                                        for k in range(vehicleNumber)]

    # vehiclesUsedNumber: LSExpression = model.sum(vehiclesUsed)

    routeDistanceList: list[Optional[LSExpression]] = [None] * vehicleNumber

    baseDistanceArray: LSExpression = model.array(newBaseDistance)

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

    # Objective: minimize the number of vehicles used, then minimize the distance traveled
    model.minimize(totalDistance)
    # model.minimize(vehiclesUsedNumber) # Não precisa minizar a quantidade de voltas

    model.close()

    # Parameterizes the solver
    localSolver.param.time_limit = timeLimit

    # Effectivelly executes the solver
    localSolver.solve()

    #
    # Writes the solution to the standard output with the following format:
    #  - total distance
    #  - for each vehicle the nodes visited (omitting the start/end)
    #
    # print(f"{vehiclesUsedNumber.value} {totalDistance.value}")
    print(f"{totalDistance.value}")

    for k in range(vehicleNumber):
      if not vehiclesUsed[k].value:
        continue

      # Values in sequence are in [0..nbCustomers-1]. +2 is to put it back in [2..nbCustomers+1]
      # as in the data files (1 being the depot)
      for customer in nodeSequence[k].value:
        sys.stdout.write(f"{customer+2}")

      print()
