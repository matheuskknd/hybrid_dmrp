#!/usr/bin/env python3.10
# -*- coding: UTF-8 -*-

from brkga_mp_ipr.algorithm import BrkgaMpIpr
from brkga_mp_ipr.types import (BrkgaParams, BaseChromosome)
from brkga_mp_ipr.enums import (BiasFunctionType, PathRelinkingSelection,
                                PathRelinkingType, Sense)
from .ConstructiveHeurisitc import generateInitialPopulation
from .VehicleRouting import (VRPSolution, getMininumVehicleRouting)
from typing import (Any, NamedTuple, Optional)

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

  def chromosome2Set(self, chromosome: BaseChromosome,
                     rewrite: bool = False) -> set[int]:

    # Auxliar list for sorting by gene value
    sortedByGeneList: list[Item] = [
      Item(i, chromosome[i]) for i in range(self.N)
    ]
    sortedByGeneList.sort(key=lambda t: t.gene, reverse=True)

    # Auxliar list for recording visited nodes
    visitedList: list[bool] = self.N * [False]

    # Set of nodes to visit
    selectedNodeSet: set[int] = set()

    if rewrite ==False:
      for i in sortedByGeneList:
        if not visitedList[i.index]:
          selectedNodeSet.add(i.index)

          # Mark i and its neighboors as visited
          visitedList[i.index] = True
          for j in self._communicationMatrix[i.index]:
            visitedList[j] = True

    else:
      for i in sortedByGeneList:
        if not visitedList[i.index]:
          selectedNodeSet.add(i.index)

          # Mark i and its neighboors as visited and rewrite the chromosome
          visitedList[i.index] = True
          chromosome[i.index] = 1

          for j in self._communicationMatrix[i.index]:
            visitedList[j] = True
            chromosome[j] = 0

    return selectedNodeSet

  def decode(self, chromosome: BaseChromosome, *,
             rewrite: bool = False) -> float:
    """Decoder interface method."""

    vrpSolution: VRPSolution = getMininumVehicleRouting(
      seed=self._seed,
      allDistances=self._allDistances,
      baseDistance=self._baseDistance,
      minimumDominatingSet=self.chromosome2Set(chromosome, rewrite),
      timeLimit=self._vrpSolverTimeLimit,
    )

    return vrpSolution.cost


class HybridBrkgaSolution(VRPSolution):
  """Class the holds the Hybrid BRKGA subproblem result."""

  def __init__(self, cost: float, *,
               minimumDominatingSet: Optional[set[int]] = None,
               evolutionPerGen: Optional[list[float]] = None,
               **kwargs: dict[str, Any]) -> None:

    # Build the parent object
    super().__init__(cost, **kwargs)

    self.minimumDominatingSet: Optional[set[int]] = minimumDominatingSet
    """The minimum dominating set corresponding to the solution with the best fitness ever found."""

    self.evolutionPerGen: Optional[list[float]] = evolutionPerGen
    """The best fitness in the population for each generation evolved."""


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

  # Use a semi-greedy heuristic to create an initial population
  ga.set_initial_population(
    generateInitialPopulation(allDistances, baseDistance, communicationMatrix,
                              seed=seed, population_size=population_size,
                              chromosome_size=len(allDistances)))

  # Run the BRKGA
  evolutionPerGen: list[float] = [INFINITY] * (num_generations+1)
  ga.initialize()

  evolutionPerGen[0] = ga.get_best_fitness()
  bestFitnessSoFar: float = evolutionPerGen[0]
  bestFitnessCount: int = 1

  for i in range(1, num_generations + 1):
    ga.evolve(num_generations=1)
    evolutionPerGen[i] = ga.get_best_fitness()

    # Debug only
    if quiet ==False:
      if i == 1:
        print(f"Semi-greedy best solution: {evolutionPerGen[0]}.")
      elif i % 50 == 0:
        print(f"Generation {i} best fitness: {evolutionPerGen[i]}.")

    if evolutionPerGen[i] < bestFitnessSoFar:
      bestFitnessSoFar = evolutionPerGen[i]
      bestFitnessCount = 1

    else:
      bestFitnessCount += 1
      assert id(evolutionPerGen[i]) == id(bestFitnessSoFar)

      # Maximum number of generations without any improvement
      if bestFitnessCount == 100:
        print(f"Generation {i}. Stopping due to stagnation since generation {i-100}.")
        break

  # Debug and evaluation
  if quiet ==False:
    mdsSolution: set[int] = decoder.chromosome2Set(ga.get_best_chromosome())
    print(f"MDS Solution: {sorted(mdsSolution)}\n")

    vrpSolution: VRPSolution = getMininumVehicleRouting(
      seed=seed, allDistances=allDistances, baseDistance=baseDistance,
      minimumDominatingSet=mdsSolution, timeLimit=vrpSolverTimeLimit,
      quiet=False)

    assert abs(vrpSolution.cost - ga.get_best_fitness()) < 0.000_001, f"{(vrpSolution.cost, ga.get_best_fitness())}"

    return HybridBrkgaSolution(minimumDominatingSet=mdsSolution,
                               evolutionPerGen=evolutionPerGen,
                               **vars(vrpSolution))

  return HybridBrkgaSolution(ga.get_best_fitness())
