#!/usr/bin/env python3.10
# -*- coding: UTF-8 -*-

from .PreProcessing import read_input
from .MinimumDominatingSet import (HybridBrkgaSolution, solveHybridBrkga)
from contextlib import redirect_stdout
from typing import (Any, TextIO)
from io import StringIO
import timeit
import sys


class HybridDMRPSolution(HybridBrkgaSolution):
  """Class the holds the Hybrid DMRP result."""

  def __init__(
    self,
    base: tuple[float, float],
    coordenates: list[tuple[float, float]],
    allDistances: list[list[float]],
    baseDistance: list[float],
    elapsedSeconds: float,
    **kwargs: dict[str, Any],
  ) -> None:

    # Build the parent object
    super().__init__(**kwargs)

    self.base: tuple[float, float] = base
    """The base location."""

    self.coordenateList: list[tuple[float, float]] = coordenates
    """Each node location (excluding the base)."""

    self.allDistances: list[list[float]] = allDistances
    """Square matrix with all distance between vertices."""

    self.baseDistance: list[float] = baseDistance
    """Array of distances from the base for each vertex."""

    self.elapsedSeconds: float = elapsedSeconds
    """The total time spent running the algorithm (seconds)."""

    self.N: int = len(allDistances)
    """Number of vertices."""


def solveHybridDMRP(instance: TextIO, *, quiet: bool, seed: int,
                    num_generations: int, population_size: int,
                    elite_percentage: float, mutants_percentage: float,
                    total_parents: int, num_elite_parents: int):

  # Disable debug printing
  devNull: StringIO = StringIO()
  devNull.write = lambda x: len(x)
  devNull.writelines = lambda x: None

  with redirect_stdout(devNull if quiet else sys.stdout):
    # Read the instance file
    (
      all_distances,
      base_distance,
      comunicacoes,
      base,
      coordenates,
    ) = read_input(instance)

    assert len(all_distances[0]) == len(base_distance)
    assert len(coordenates) == len(base_distance)

    # Account the time spent running the metaheuristics
    startTime: float = timeit.default_timer()

    # Execute the hybrid heuristics
    hybridBrkgaSolution: HybridBrkgaSolution = solveHybridBrkga(
      all_distances, base_distance, comunicacoes, seed=seed,
      num_generations=num_generations, population_size=population_size,
      elite_percentage=elite_percentage, mutants_percentage=mutants_percentage,
      total_parents=total_parents, num_elite_parents=num_elite_parents,
      quiet=quiet)

    endTime: float = timeit.default_timer()

  # Return the solution and statistics for the caller
  return HybridDMRPSolution(base=base, coordenates=coordenates,
                            allDistances=all_distances,
                            baseDistance=base_distance,
                            elapsedSeconds=endTime - startTime,
                            **vars(hybridBrkgaSolution))
