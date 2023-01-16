#!/usr/bin/env python3.10
# -*- coding: UTF-8 -*-

from .ConstructiveHeurisitc import (
  RclItem,
  generateInitialPopulation,
  generateRclBase,
)

from brkga_mp_ipr.algorithm import BrkgaMpIpr
from brkga_mp_ipr.types import (BrkgaParams, BaseChromosome)
from brkga_mp_ipr.enums import (BiasFunctionType, PathRelinkingSelection,
                                PathRelinkingType, Sense)
from .VehicleRouting import (VRPSolution, getMininumVehicleRouting)
from typing import (Any, NamedTuple)
import timeit

INFINITY: float = float("inf")


class Item(NamedTuple):
  index: int
  """Vertex index or (ID - 1). Belongs to [0,N-1]."""

  gene: float
  """Corresponding gene value. Belogs to [0,1]."""


class BRKGADecoder:

  def __init__(self, seed: int, allDistances: list[list[float]],
               baseDistance: list[float], communicationMatrix: list[list[int]],
               *, vrpSolverTimeLimit: float) -> None:

    self._seed: int = seed
    """Pseudo-random number generator seed."""

    self._allDistances: list[list[float]] = allDistances
    """Square matrix with all distance between vertices."""

    self._baseDistance: list[float] = baseDistance
    """Array of distances from the base for each vertex."""

    self._communicationMatrix: list[list[int]] = communicationMatrix
    """Sparse matrix containing the reach graph."""

    self._vrpSolverTimeLimit: float = vrpSolverTimeLimit
    """Time limit to spend running the VRP solver at each decoding operation."""

    self.N: int = len(communicationMatrix)
    """Number of vertices."""

    self._bestVrpSolution: VRPSolution = VRPSolution(INFINITY)
    """Cached best VRP solution ever found while decoding."""

    self._bestVrpSolutionDms: set[int] = set()
    """Cached minimum dominating set corresponding to cached VRP solution."""

  def chromosome2Set(self, chromosome: BaseChromosome) -> set[int]:

    # Auxliar list for sorting by gene value
    sortedByGeneList: list[Item] = [
      Item(i, chromosome[i]) for i in range(self.N)
    ]
    sortedByGeneList.sort(key=lambda t: t.gene, reverse=True)

    # Auxliar list for recording visited nodes
    visitedList: list[bool] = self.N * [False]

    # Set of nodes to visit
    selectedNodeSet: set[int] = set()

    for i in sortedByGeneList:
      if not visitedList[i.index]:
        selectedNodeSet.add(i.index)

        # Mark i and its neighboors as visited
        visitedList[i.index] = True
        for j in self._communicationMatrix[i.index]:
          visitedList[j] = True

    return selectedNodeSet

  def decode(self, chromosome: BaseChromosome, *,
             rewrite: bool = False) -> float:
    """Decoder interface method."""

    # Calculate the minimum dominating set
    minimumDominatingSet: set[int] = self.chromosome2Set(chromosome)

    # Find a VRP subproblem solution
    vrpSolution: VRPSolution = getMininumVehicleRouting(
      seed=self._seed,
      allDistances=self._allDistances,
      baseDistance=self._baseDistance,
      minimumDominatingSet=minimumDominatingSet,
      timeLimit=self._vrpSolverTimeLimit,
      bestVrpCost=self._bestVrpSolution.vrpCost,
    )

    # Save the best solution ever found
    if vrpSolution.vrpCost < self._bestVrpSolution.vrpCost:
      self._bestVrpSolutionDms = minimumDominatingSet
      self._bestVrpSolution = vrpSolution

    # Return the VRP solution cost as fitness
    return vrpSolution.vrpCost

  def getBestEverFound(self) -> tuple[VRPSolution, set[int]]:
    """Return a tuple with the best VRP solution ever found with the corresponding MDS."""
    return (self._bestVrpSolution, self._bestVrpSolutionDms)


class HybridBrkgaSolution(VRPSolution):
  """Class the holds the Hybrid BRKGA subproblem result."""

  def __init__(self, *, minimumDominatingSet: set[int] = set(),
               evolutionPerGen: list[float] = list(),
               gaElapsedSeconds: float = 0.0, **kwargs: dict[str, Any]) -> None:

    # Build the parent object
    super().__init__(**kwargs)

    self.minimumDominatingSet: set[int] = minimumDominatingSet
    """The minimum dominating set corresponding to the solution with the best fitness ever found."""

    self.evolutionPerGen: list[float] = evolutionPerGen
    """The best fitness in the population for each generation evolved."""

    self.gaElapsedSeconds: float = gaElapsedSeconds
    """The time spent running the Hybrid BRKGA algorith (seconds)."""


def solveHybridBrkga(allDistances: list[list[float]], baseDistance: list[float],
                     communicationMatrix: list[list[int]], *, seed: int,
                     num_generations: int, population_size: int,
                     elite_percentage: float, mutants_percentage: float,
                     total_parents: int, num_elite_parents: int,
                     vrpSolverTimeLimit: float = 5,
                     quiet: bool = False) -> HybridBrkgaSolution:

  # BRKGA decoder
  decoder: BRKGADecoder = BRKGADecoder(seed, allDistances, baseDistance,
                                       communicationMatrix,
                                       vrpSolverTimeLimit=vrpSolverTimeLimit)

  # BRKGA Hyper-parameters
  params: BrkgaParams = BrkgaParams()
  params.population_size = population_size
  params.elite_percentage = elite_percentage
  params.mutants_percentage = mutants_percentage
  params.num_elite_parents = num_elite_parents
  params.total_parents = total_parents
  params.bias_type = BiasFunctionType.CONSTANT  # Fixed
  params.num_independent_populations = 1  # Exchange not implemented yet

  # Path Relinking parameters
  params.pr_number_pairs = 0  # Path Relinking not implemented yet
  params.pr_minimum_distance = 0.0  # Path Relinking not implemented yet
  params.pr_type = PathRelinkingType.DIRECT  # Path Relinking not implemented yet
  params.pr_selection = PathRelinkingSelection.BESTSOLUTION  # Path Relinking not implemented yet
  params.alpha_block_size = 0.0  # Path Relinking not implemented yet
  params.pr_percentage = 0.0  # Path Relinking not implemented yet

  # BRKGA object
  ga: BrkgaMpIpr = BrkgaMpIpr(decoder, Sense.MINIMIZE, seed=seed,
                              chromosome_size=len(communicationMatrix),
                              params=params)

  # Generate the RCL base list
  rclBaseList: list[RclItem] = generateRclBase(allDistances,
                                               communicationMatrix)

  # Account the time spent running the BRKGA
  startTime: float = timeit.default_timer()

  # Use a semi-greedy heuristic to create an initial population
  ga.set_initial_population(
    generateInitialPopulation(allDistances, baseDistance, communicationMatrix,
                              rclBaseList, seed=seed,
                              population_size=population_size,
                              chromosome_size=len(allDistances)))

  # Run the BRKGA
  evolutionPerGen: list[float] = [INFINITY] * (num_generations+1)
  ga.initialize()

  evolutionPerGen[0] = ga.get_best_fitness()
  TIME_LIMIT: int = 5 * 60  # Up to 5 minutes

  for i in range(1, num_generations + 1):
    ga.evolve(num_generations=1)
    evolutionPerGen[i] = ga.get_best_fitness()

    # Debug only
    if quiet ==False:
      if i == 1:
        print(f"Semi-greedy best solution: {evolutionPerGen[0]:.2f}.")
      elif i % 50 == 0:
        print(f"Generation {i} best fitness: {evolutionPerGen[i]:.2f}.")

    # Maximum time to spend on the BRKGA reached
    if timeit.default_timer() - startTime > TIME_LIMIT:
      print(f"Generation {i}. Stopping due to time limit "
            f"exceeded: {timeit.default_timer() - startTime:.2f} > {TIME_LIMIT}.")

      evolutionPerGen = evolutionPerGen[:i + 1]
      break

  # Retrieve the best solution ever found
  (bestVrpSolution, bestMds) = decoder.getBestEverFound()
  endTime: float = timeit.default_timer()

  # Debug
  if not quiet:
    mdsSolution: set[int] = decoder.chromosome2Set(ga.get_best_chromosome())
    print(f"MDS Solution: {sorted(mdsSolution)}\n")
    assert bestMds == mdsSolution

    vrpSolution: VRPSolution = getMininumVehicleRouting(
      seed=seed, allDistances=allDistances, baseDistance=baseDistance,
      minimumDominatingSet=mdsSolution, timeLimit=vrpSolverTimeLimit,
      quiet=False)

    assert abs(vrpSolution.vrpCost - ga.get_best_fitness()) < 0.000_001, f"{(vrpSolution.vrpCost, ga.get_best_fitness())}"
    assert abs(vrpSolution.vrpCost - bestVrpSolution.vrpCost) < 0.000_001, f"{(vrpSolution.vrpCost, bestVrpSolution.vrpCost)}"
    del vrpSolution
    del mdsSolution
    print()

  return HybridBrkgaSolution(minimumDominatingSet=bestMds,
                             evolutionPerGen=evolutionPerGen,
                             gaElapsedSeconds=endTime - startTime,
                             **vars(bestVrpSolution))
