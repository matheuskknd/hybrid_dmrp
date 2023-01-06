#!/usr/bin/env python3.10
# -*- coding: UTF-8 -*-

from argparse import (ArgumentParser, FileType, Namespace)
import random

from .api import (HybridDMRPSolution, solveHybridDMRP)


# Runs the program
def main() -> None:
  try:
    argParser: ArgumentParser = ArgumentParser(prog="hybrid_dmrp")

    # Parameter parsedArgs.instance
    _ = argParser.add_argument("instance", type=FileType("r", encoding="UTF-8"),
                               metavar="instancePath",
                               help="The input file path. Must be readable.")

    # Parameter parsedArgs.quiet
    _ = argParser.add_argument("-q", "--quiet", default=False, dest="quiet",
                               action="store_true",
                               help="Silence the execution. Defaults to False.")

    # Parameter parsedArgs.seed
    _ = argParser.add_argument(
      "--seed", type=int, default=random.randint(1, 1000), dest="seed",
      metavar="randomSeed",
      help="Seed for the pseudo random number generator. Default random.")

    # Parameter parsedArgs.num_generations
    _ = argParser.add_argument(
      "--num-generations", type=int, default=10, dest="num_generations",
      metavar="num_generations",
      help="Number of generations for the BRKGA to evolve. Default 10.")

    # Parameter parsedArgs.population_size
    _ = argParser.add_argument(
      "--population-size", type=int, default=25, dest="population_size",
      metavar="population_size",
      help="Number of population individuals for the BRKGA. Default 25.")

    # Parameter parsedArgs.elite_percentage
    _ = argParser.add_argument(
      "--elite-percentage", type=float, default=0.10, dest="elite_percentage",
      metavar="elite_percentage",
      help="Percentage of elit individuals on BRKGA populations. Default 0.10.")

    # Parameter parsedArgs.mutants_percentage
    _ = argParser.add_argument(
      "--mutants-percentage", type=float, default=0.10,
      dest="mutants_percentage", metavar="mutants_percentage",
      help="Percentage of mutant individuals on BRKGA populations. Default 0.10.")

    # Parameter parsedArgs.total_parents
    _ = argParser.add_argument(
      "--total-parents", type=int, default=2, dest="total_parents",
      metavar="total_parents",
      help="Number of parents in the BRKGA crossover operator. Default 2.")

    # Parameter parsedArgs.num_elite_parents
    _ = argParser.add_argument(
      "--num-elite-parents", type=int, default=1, dest="num_elite_parents",
      metavar="num_elite_parents",
      help="Number of elit parents in the BRKGA crossover operator. Default 1.")

    parsedArgs: Namespace = argParser.parse_args()

    # Debug for the input parameters
    # with open("debug_params.txt", "a") as debug_params:
    #   debug_params.write(str(parsedArgs) + "\n")

    # Effectivelly executes the main function
    solution: HybridDMRPSolution = solveHybridDMRP(
      parsedArgs.instance,
      quiet=parsedArgs.quiet,
      seed=parsedArgs.seed,
      num_generations=parsedArgs.num_generations,
      population_size=parsedArgs.population_size,
      elite_percentage=parsedArgs.elite_percentage,
      mutants_percentage=parsedArgs.mutants_percentage,
      total_parents=parsedArgs.total_parents,
      num_elite_parents=parsedArgs.num_elite_parents,
      isCLI=True,
    )

    # Required for IRace - IT MUST BE THE LAST THING PRINTED
    print(f"{solution.babCost:.2f} {solution.elapsedSeconds:.2f}")

  except BaseException as e:
    raise e
