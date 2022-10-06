#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

from brkga_mp_ipr.algorithm import BrkgaMpIpr
from brkga_mp_ipr.types import BrkgaParams
from brkga_mp_ipr.enums import (BiasFunctionType, PathRelinkingSelection,
                                PathRelinkingType, Sense)
import random

from .BRKGADecoder import Decoder


def main():
  sampleReachMatrix: list[list[int]] = [
    [2],
    [2],
    [0, 1],
  ]

  # BRKGA decoder
  decoder: Decoder = Decoder(sampleReachMatrix)

  # BRKGA parameters
  params: BrkgaParams = BrkgaParams()
  params.population_size = 100  # Try it out
  params.elite_percentage = 0.3  # Try it out
  params.mutants_percentage = 0.2  # Try it out
  params.num_elite_parents = 1  # Try it out
  params.total_parents = 2  # Try it out
  params.bias_type = BiasFunctionType.CONSTANT  # Try it out
  params.num_independent_populations = 1  # Exchange not implemented yet
  params.pr_number_pairs = 0  # Try it out
  params.pr_minimum_distance = 0.0  # Try it out
  params.pr_type = PathRelinkingType.DIRECT  # Try it out
  params.pr_selection = PathRelinkingSelection.BESTSOLUTION  # Try it out
  params.alpha_block_size = 0.0  # Try it out
  params.pr_percentage = 0.0  # Try it out

  # BRKGA object
  ga: BrkgaMpIpr = BrkgaMpIpr(decoder, Sense.MINIMIZE,
                              seed=random.randint(1, 1000),
                              chromosome_size=len(sampleReachMatrix),
                              params=params)

  # Run the BRKGA
  ga.initialize()
  ga.evolve(num_generations=1_000)  # Try it out
  bestSoFar: set[int] = decoder.chromosome2Set(ga.get_best_chromosome())

  # Debug - print the best solution ever found
  print(bestSoFar)


# Execute the main function
if __name__ == "__main__":
  main()
