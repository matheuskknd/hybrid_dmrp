#!/usr/bin/env python3.10
# -*- coding: UTF-8 -*-

from hybrid_dmrp import (HybridDMRPSolution, solveHybridDMRP, CplexStatusCodeEnum)
from os.path import (abspath, basename, dirname, join, exists)
from contextlib import (redirect_stdout, redirect_stderr)
from adjustText import adjust_text
from networkx import MultiDiGraph
from localsolver import LSEnum
from matplotlib import pyplot
from pandas import DataFrame
from typing import Any
from enum import Enum
import traceback
import random
import timeit
import numpy
import glob
import json
import os


def jsonSerializer(o: Any) -> Any:
  if isinstance(o, MultiDiGraph):
    return [(u, v, ddict) for (u, v, ddict) in o.edges(data=True)]
  elif isinstance(o, HybridDMRPSolution):
    result: dict[str, Any] = vars(o)
    result["minimumDominatingSet"] = sorted(result["minimumDominatingSet"])
    return result
  elif isinstance(o, (Enum, LSEnum)):
    return o.name
  else:
    assert False, f"Unexpected type: {type(o)}."


def evoCurvePlot(solution: HybridDMRPSolution, evoCurveFileName: str) -> None:

  # The BRKGA evolution curve
  pyplot.plot(solution.evolutionPerGen)

  # Draw some the interesting points
  semiGreedyBest: tuple[int, float] = (0, solution.evolutionPerGen[0])
  brkgaBest: tuple[int, float] = (len(solution.evolutionPerGen) - 1,
                                  solution.evolutionPerGen[-1])

  textList: list[pyplot.Text] = [
    pyplot.text(semiGreedyBest[0], semiGreedyBest[1],
                f"SGC*={round(solution.evolutionPerGen[0],1)}",
                fontdict={"size": "small"}),
    pyplot.text(brkgaBest[0], brkgaBest[1],
                f"BRKGA*={round(solution.evolutionPerGen[-1],1)}",
                fontdict={"size": "small"}),
  ]

  pyplot.plot(semiGreedyBest[0], semiGreedyBest[1], c="b", marker="s")
  pyplot.plot(brkgaBest[0], brkgaBest[1], c="b", marker="s")

  # Set the labels names, title and finally save it
  pyplot.title("BRKGA best fitness per generation")
  pyplot.ylabel("Best fitness")
  pyplot.xlabel("Generation")

  # Ref: https://stackoverflow.com/a/34762716
  """
  adjust_text(
    textList,
    x=[semiGreedyBest[0], brkgaBest[0]],
    y=[semiGreedyBest[1], brkgaBest[1]],
    autoalign=True,
    arrowprops={
      "arrowstyle": "-",
      "color": "r",
      "lw": 0.3,
    },
    avoid_self=False,
    force_text=0.01,
    force_points=0.01,
    force_objects=0.01,
    lim=1,
  )
  """

  pyplot.savefig(evoCurveFileName)
  pyplot.clf()
  pyplot.cla()


def solutionScatterPlot(solution: HybridDMRPSolution, cost: float,
                        graph: MultiDiGraph, scatterFileName: str) -> None:

  # List of coordinates of all nodes (including the base)
  locX: list[float] = ([solution.base[0]] +
                       [node[0] for node in solution.coordenateList])

  locY: list[float] = ([solution.base[1]] +
                       [node[1] for node in solution.coordenateList])

  # All vehicles have the same autonomy (travelling capacity)
  autonomy: float = 2 * max(solution.baseDistance[i]
                            for i in solution.minimumDominatingSet)

  # Add a title
  pyplot.title("Solution\n"
               f"N = {solution.N}"
               f", |MDS|= {len(solution.minimumDominatingSet)}"
               f", autonomy={round(autonomy,2)}"
               f", totalCost={round(cost,2)}")

  # Draw the nodes location (except the base)
  # Visited nodes are blue
  pyplot.scatter(
    [locX[i + 1] for i in solution.minimumDominatingSet],
    [locY[i + 1] for i in solution.minimumDominatingSet],
    c="#0000ff",
  )

  # Non visited nodes are black
  nSet: frozenset[int] = frozenset(range(solution.N))
  pyplot.scatter(
    [locX[i + 1] for i in nSet.difference(solution.minimumDominatingSet)],
    [locY[i + 1] for i in nSet.difference(solution.minimumDominatingSet)],
    c="#000000",
    alpha=0.5,
  )

  # Draw all active edges
  textList: list[pyplot.Text] = []

  edgeSet: frozenset[tuple[int, int]] = frozenset(
    (u, v) for (u, v) in graph.edges(data=False))

  for (u, v, ddict) in graph.edges(data=True):
    assert "cost" in ddict, f"Missing property 'cost' on edge {(u,v)}."

    # Should not draw some edges "twice"
    if u > v and (v, u) in edgeSet:
      continue

    # Draw the edge
    pyplot.plot([locX[u], locX[v]], [locY[u], locY[v]], c="#3ac14a", alpha=0.3)

    # Should not draw the edge costs on big graphs
    if solution.N > 50:
      continue

    textList.append(
      pyplot.text((locX[u] + locX[v]) / 2, (locY[u] + locY[v]) / 2,
                  f"$d_{{{u},{v}}}={round(ddict['cost'],1)}$",
                  fontdict={"size": "x-small"}),
    )

  # Draw the base node location
  pyplot.plot(locX[0], locY[0], c="#ff0000", marker="s")

  # Calculate the ticks
  xticks: list[float] = [
    round(min(locX), 4),
    round((min(locX) + max(locX)) / 2, 4),
    round(max(locX), 4)
  ]

  yticks: list[float] = [
    round(min(locY), 4),
    round((min(locY) + max(locY)) / 2, 4),
    round(max(locY), 4)
  ]

  # Set the labels names, title and finally save it
  pyplot.ylabel("Latitude")
  pyplot.xlabel("Longitude")
  pyplot.xticks(xticks)
  pyplot.yticks(yticks)

  # Ref: https://stackoverflow.com/a/34762716
  if len(textList) != 0:
    adjust_text(
      textList,
      x=locX,
      y=locY,
      autoalign=True,
      arrowprops={
        "arrowstyle": "-",
        "color": "r",
        "lw": 0.3,
      },
      avoid_self=False,
      expand_points=(0.003, 0.003),
      force_text=(0.1, 0.1),
      force_points=(0.01, 0.02),
    )

  pyplot.savefig(scatterFileName)
  pyplot.clf()
  pyplot.cla()


# Change for this script execution path
workingDir: str = dirname(abspath(__file__))
os.chdir(workingDir)

# Create a directory to store the results
resultDirName: str = str(timeit.default_timer())
print(f"Storing the results in: {resultDirName}.")
os.mkdir(resultDirName)

# Get the list of instances to run
instanceDir: str = join(workingDir, "instances")
_instanceFileNameList: list[str] = glob.glob(
  join(instanceDir, "_").removesuffix("_") + "*.csv",
)

instanceFileNameList: list[tuple[str, int]] = []

for instance in _instanceFileNameList:
  with open(instance, "r", encoding="UTF-8") as instanceFile:
    instanceFileNameList.append((instance, int(instanceFile.readline())))

instanceFileNameList = sorted(instanceFileNameList, key=lambda pair: pair[1])
del _instanceFileNameList
del instanceFile
del instanceDir
del workingDir
del instance


def main():

  # Overall data statistics
  # statisticsDict: dict[str, list[Any]] = {
  #   "instance": [],
  #   "min_cost": [],
  #   "max_cost": [],
  #   "mean_cost": [],
  #   "min_time": [],
  #   "max_time": [],
  #   "mean_time": [],
  # }

  # Number of rounds
  NB_SAMPLES: int = 3

  # Auxliar variables
  # costSamples: list[float] = [0.0] * NB_SAMPLES
  # timeSamples: list[float] = [0.0] * NB_SAMPLES

  # Run the algorithm NB_SAMPLES times for each instance with the same parameters
  for run in range(NB_SAMPLES):
    for (instance, _) in instanceFileNameList:
      instanceBaseName: str = basename(instance).removesuffix(".csv")
      instanceResultDir: str = join(resultDirName, instanceBaseName)

      # Result file names
      evoCurveFileName: str = join(instanceResultDir, f"evo_curve_{run}.png")
      solutionFileName: str = join(instanceResultDir, f"solution_{run}.json")
      vrpScatterFileName: str = join(instanceResultDir,
                                     f"vrp_scatter_{run}.png")
      babScatterFileName: str = join(instanceResultDir,
                                     f"bab_scatter_{run}.png")

      with open(instance, "r", encoding="UTF-8") as instanceFile:
        try:
          seed: int = random.randint(1, 1000)
          print("\n########")
          print(f"(Seed: {seed}, Run: {run}, Instance: {instanceBaseName})\n")

          hybridDMRPSolution: HybridDMRPSolution = solveHybridDMRP(
            instanceFile,
            quiet=False,
            seed=seed,
            num_generations=200,
            population_size=25,
            elite_percentage=0.15,
            mutants_percentage=0.22,
            total_parents=3,
            num_elite_parents=2,
          )

        except KeyboardInterrupt:
          print(f"\nKeyboardInterrupt in {run} of {instanceBaseName}")
          exit(1)

        except BaseException as e:
          print(f"\nErro in {run} of {instanceBaseName}:\n{e}\n")
          print(traceback.format_exc())
          continue

      # Create the result directory on success
      if not exists(instanceResultDir):
        os.mkdir(instanceResultDir)

      # Save the best fitness per generation curve
      evoCurvePlot(hybridDMRPSolution, evoCurveFileName)

      # Save the best solution as a scatter plot
      solutionScatterPlot(
        hybridDMRPSolution,
        hybridDMRPSolution.vrpCost,
        hybridDMRPSolution.vrpGraph,
        vrpScatterFileName,
      )

      solutionScatterPlot(
        hybridDMRPSolution,
        hybridDMRPSolution.babCost,
        hybridDMRPSolution.babGraph,
        babScatterFileName,
      )

      # Save the result for statistics
      # timeSamples[run] = hybridDMRPSolution.elapsedSeconds
      # costSamples[run] = hybridDMRPSolution.vrpCost

      # Save the result object (serialized)
      with open(solutionFileName, "w", encoding="UTF-8") as solutionFile:

        # Remove instance data and already plotted data
        del hybridDMRPSolution.allDistances
        del hybridDMRPSolution.baseDistance
        #del hybridDMRPSolution.evolutionPerGen
        del hybridDMRPSolution.coordenateList
        del hybridDMRPSolution.base

        json.dump(
          hybridDMRPSolution,
          solutionFile,
          ensure_ascii=False,
          indent=2,
          default=jsonSerializer,
        )

      # End of the inner most loop
      if hybridDMRPSolution.babStatusCode in {
          CplexStatusCodeEnum.CPXMIP_ABORT_FEAS,
          CplexStatusCodeEnum.CPXMIP_ABORT_INFEAS,
          CplexStatusCodeEnum.CPXMIP_ABORT_RELAXATION_UNBOUNDED,
          CplexStatusCodeEnum.CPXMIP_ABORT_RELAXED,
          CplexStatusCodeEnum.CPX_STAT_ABORT_USER,
          CplexStatusCodeEnum.CPX_STAT_CONFLICT_ABORT_CONTRADICTION,
          CplexStatusCodeEnum.CPX_STAT_CONFLICT_ABORT_USER,
      }:
        raise Exception("The last 'babstatusCode' "
                        f"({hybridDMRPSolution.babStatusCode.name})"
                        " requires a complete halt.")

      del hybridDMRPSolution

    # Save some statistics
    # statisticsDict["instance"].append(instanceBaseName)
    # statisticsDict["min_cost"].append(numpy.min(costSamples))
    # statisticsDict["max_cost"].append(numpy.max(costSamples))
    # statisticsDict["mean_cost"].append(numpy.mean(costSamples))
    # statisticsDict["min_time"].append(numpy.min(timeSamples))
    # statisticsDict["max_time"].append(numpy.max(timeSamples))
    # statisticsDict["mean_time"].append(numpy.mean(timeSamples))

    # Convert the collected samples to a pandas DataFrame and save it into a CSV file
    # DataFrame(data={
    #   "cost": costSamples,
    #   "seconds": timeSamples,
    # }).to_csv(
    #   join(instanceResultDir, "summary.csv"),
    #   sep="\t",
    #   index=False,
    #   encoding="UTF-8",
    # )

  # Convert the Overall data statistics to a pandas DataFrame and save it into a CSV file
  # DataFrame(data=statisticsDict).to_csv(
  #   join(resultDirName, "summary.csv"),
  #   sep="\t",
  #   index=False,
  #   encoding="UTF-8",
  # )


stdoutLogName: str = join(resultDirName, "stdout.txt")
stderrLogName: str = join(resultDirName, "stderr.txt")

with open(stdoutLogName, "w+", encoding="UTF-8") as stdoutLog:
  with open(stderrLogName, "w+", encoding="UTF-8") as stderrLog:
    with redirect_stdout(stdoutLog):
      with redirect_stderr(stderrLog):
        main()
