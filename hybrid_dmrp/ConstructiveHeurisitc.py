#!/usr/bin/env python3.10
# -*- coding: UTF-8 -*-

from brkga_mp_ipr.types import BaseChromosome
from networkx import (Graph, eigenvector_centrality)
from typing import (NamedTuple, Optional, cast)
from pyllist import (dllist, dllistnode)
import random
import timeit

INFINITY: float = float("inf")


class RclItem(NamedTuple):
  index: int
  """Vertex index/ID."""

  nodeEC: float
  """Eigenvector centrality value of the vertex."""


def generateRclBase(allDistances: list[list[float]],
                    communicationMatrix: list[list[int]]) -> list[RclItem]:
  """
  Generates a set from which the initial candidate lists will be generated from.

  The set is composed by the original vertex index (or ID - 1) and its respective cost,
  calculated used the Eigenvector Centrality algorithm.
  """

  # Convert sparse matrix into a networkx.Graph object
  edges: list[tuple[int, int, dict[str, float]]] = []
  loneNodeList: dllist = dllist()

  for i in range(len(communicationMatrix)):
    edges.extend((
      i,
      j,
      {
        "weight": allDistances[i][j]
      },
    ) for j in communicationMatrix[i] if j > i)

    if len(communicationMatrix[i]) == 0:
      loneNodeList.append(i)

  # Calculate the eigenvector centrality for each node
  # https://networkx.org/documentation/stable/reference/algorithms/generated/networkx.algorithms.centrality.eigenvector_centrality.html
  graph: Graph = Graph(edges)
  graph.add_nodes_from(loneNodeList)

  ecDict: dict[int, float] = eigenvector_centrality(
    graph,
    max_iter=100_000,
    tol=0.000_001,
    nstart=None,
    weight="weight",
  )

  # Calculate the result
  return [RclItem(i, ecDict[i]) for i in range(len(communicationMatrix))]


def generateInitialPopulation(allDistances: list[list[float]],
                              baseDistance: list[float],
                              communicationMatrix: list[list[int]], *, seed: int,
                              population_size: int, chromosome_size: int,
                              alpha: float = 0.5) -> list[BaseChromosome]:
  """
  Use a semi-greedy constructive heuristic to initialize a population of population_size individuals.
  """

  assert chromosome_size == len(communicationMatrix)
  assert chromosome_size == len(allDistances)
  assert chromosome_size == len(baseDistance)

  print("Running the semi-greedy constructive... ", end="")
  startTime: float = timeit.default_timer()

  # Set the random seed before doing anything
  random.seed(seed)

  # Generate the RCL base list
  rclBaseList: list[RclItem] = generateRclBase(allDistances,
                                               communicationMatrix)

  # Auxiliar variables
  _communicationMatrix: list[frozenset[int]] = [
    frozenset(i) for i in communicationMatrix
  ]

  def createChromosome() -> BaseChromosome:
    # Shuffle the RCL base list
    candidateList: list[RclItem] = rclBaseList.copy()
    random.shuffle(candidateList)

    _rclBaseList: dllist = dllist(candidateList)
    del candidateList

    # Generate a new chromosome
    chromosome: BaseChromosome = BaseChromosome(None
                                                for _ in range(chromosome_size))

    removedSet: set[int] = set()

    while len(_rclBaseList) != 0:
      maxEC: float = -INFINITY
      minEC: float = INFINITY

      for item in _rclBaseList:
        maxEC = max(maxEC, item.nodeEC)
        minEC = min(minEC, item.nodeEC)

      # Find any random item that belongs to the RCL
      minAllowedEC: float = minEC + alpha * (maxEC-minEC)
      chosenItem: Optional[RclItem] = None

      node: dllistnode = _rclBaseList.nodeat(0)
      while True:
        if cast(RclItem, node.value).nodeEC > minAllowedEC - 0.000_001:
          chosenItem: RclItem = _rclBaseList.remove(node)
          chromosome[chosenItem.index] = 1
          removedSet.add(chosenItem.index)
          break

        node = node.next

      assert isinstance(chosenItem, RclItem)

      # Remove all neighbors, so they won't enter the RCL in the future
      neighbors: frozenset[int] = _communicationMatrix[chosenItem.index]
      neighborsToRemove: int = len(neighbors.difference(removedSet))
      if neighborsToRemove == 0:
        continue

      node = _rclBaseList.nodeat(0)
      removed: int = 0

      while removed != neighborsToRemove:
        if cast(RclItem, node.value).index in neighbors:
          nodeToRemove: dllistnode = node
          node = node.next

          neighbor: RclItem = _rclBaseList.remove(nodeToRemove)
          chromosome[neighbor.index] = 0
          removedSet.add(neighbor.index)
          removed += 1

        else:
          node = node.next

    # Debug only
    for i in chromosome:
      assert i != None

    return chromosome

  # Generate the chromosomes
  firstPopulation: list[BaseChromosome] = [
    createChromosome() for _ in range(population_size)
  ]

  print(f"Done after: {timeit.default_timer() - startTime:.2f} s")
  return firstPopulation
