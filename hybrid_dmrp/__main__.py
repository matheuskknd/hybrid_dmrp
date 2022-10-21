#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import sys
from brkga_mp_ipr.algorithm import BrkgaMpIpr
from brkga_mp_ipr.types import BrkgaParams
from brkga_mp_ipr.enums import (BiasFunctionType, PathRelinkingSelection,
                                PathRelinkingType, Sense)
import random
from .PreProcessing import read_input
from .BRKGADecoder import Decoder


def main(instance_file, str_nb_drones):
  nb_trucks = int(str_nb_drones)

  (all_distances, base_distance, comunicacoes) = read_input(instance_file)
  minimumDominatingSet: set[int] = getMinimumDominatingSet(comunicacoes)

  # Required for IRace - IT MUST BE THE LAST THING PRINTED
  print(f"{float(len(minimumDominatingSet))}")


def getMinimumDominatingSet(communicationMatrix: list[list[int]]) -> set[int]:
  # BRKGA decoder
  decoder: Decoder = Decoder(communicationMatrix)

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
                              chromosome_size=len(communicationMatrix),
                              params=params)

  # Run the BRKGA
  ga.initialize()
  ga.evolve(num_generations=1_000)  # Try it out
  bestSoFar: set[int] = decoder.chromosome2Set(ga.get_best_chromosome())

  # Debug - print the best solution ever found
  print("Minimum dominating set: ", bestSoFar)
  return bestSoFar


# Execute the main function
if __name__ == '__main__':
  # Debug for the input parameters
  # with open("debug_params.txt","a") as debug_params:
  #   debug_params.write(str(sys.argv) + "\n")

  if len(sys.argv) < 2:
    print("Usage: python pcvr.py input_file [output_file] [time_limit] [nb_trucks]")
    sys.exit(1)

  instance_file = sys.argv[1]
  print("Utilizando arquivo de entrada: ", instance_file)
  print()
  main(instance_file, "1")
