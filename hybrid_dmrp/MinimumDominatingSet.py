#!/usr/bin/env python3.10
# -*- coding: UTF-8 -*-

from brkga_mp_ipr.algorithm import BrkgaMpIpr
from brkga_mp_ipr.types import (BrkgaParams, BaseChromosome)
from brkga_mp_ipr.enums import (BiasFunctionType, PathRelinkingSelection,
                                PathRelinkingType, Sense)
from typing import NamedTuple


class Item(NamedTuple):
  index: int
  gene: float


class BRKGADecoder:

  def __init__(self, reachMatrix: list[list[int]]) -> None:

    self._reachMatrix: list[list[int]] = reachMatrix
    """Sparse matrix containing the reach graph."""

    self.N: int = len(reachMatrix)
    """Number of vertex."""

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
        for j in self._reachMatrix[i.index]:
          visitedList[j] = True

    return selectedNodeSet

  def decode(self, chromosome: BaseChromosome, *,
             rewrite: bool = False) -> float:
    """Decoder interface method."""

    return len(self.chromosome2Set(chromosome))


def getMinimumDominatingSet(communicationMatrix: list[list[int]], *, seed: int,
                            num_generations: int, population_size: int,
                            elite_percentage: float, mutants_percentage: float,
                            total_parents: int,
                            num_elite_parents: int) -> set[int]:
  # BRKGA decoder
  decoder: BRKGADecoder = BRKGADecoder(communicationMatrix)

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

  # Run the BRKGA
  ga.initialize()
  ga.evolve(num_generations=num_generations)
  bestSoFar: set[int] = decoder.chromosome2Set(ga.get_best_chromosome())

  # Debug - print the best MDS solution ever found
  print("Minimum dominating set: ", bestSoFar)
  return bestSoFar
