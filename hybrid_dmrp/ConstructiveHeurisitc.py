#!/usr/bin/env python3.10
# -*- coding: UTF-8 -*-

from brkga_mp_ipr.types import BaseChromosome
from networkx import (Graph, eigenvector_centrality)
from typing import (NamedTuple, Optional, cast)
from pyllist import (dllist, dllistnode)
import random

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

  The set is composed by the original vertex index (or ID - 1) and it's respective cost,
  calculated used the Eigenvector Centrality algorithm.
  """

  # Convert sparse matrix into a networkx.Graph object
  edges: list[tuple[int, int, dict[str, float]]] = []
  graph: Graph = Graph(edges)

  for i in range(len(communicationMatrix)):
    edges.extend((
      i,
      j,
      {
        "weight": allDistances[i][j]
      },
    ) for j in communicationMatrix[i] if j > i)

    if len(communicationMatrix[i]) == 0:
      graph.add_node(i)

  # Calculate the eigenvector centrality for each node
  # https://networkx.org/documentation/stable/reference/algorithms/generated/networkx.algorithms.centrality.eigenvector_centrality.html
  graph = Graph(edges)

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
    candidateList = rclBaseList.copy()
    random.shuffle(candidateList)

    _rclBaseList: dllist = dllist(candidateList)

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

      item: Optional[dllistnode] = _rclBaseList.nodeat(0)
      while True:
        if item.value.nodeEC > minAllowedEC or abs(item.value.nodeEC - minAllowedEC) < 0.000_001:
          chosenItem = _rclBaseList.remove(item)
          chromosome[chosenItem.index] = 1
          removedSet.add(chosenItem.index)
          break

        item = item.next

      assert isinstance(chosenItem, RclItem)

      # Remove all neighbors, so they won't enter the RCL in the future
      neighbors: frozenset[int] = _communicationMatrix[chosenItem.index]
      neighborsToRemove: int = len(neighbors.difference(removedSet))
      if neighborsToRemove == 0:
        continue

      item = _rclBaseList.nodeat(0)
      removed: int = 0

      while removed != neighborsToRemove:
        if cast(RclItem, item.value).index in neighbors:
          itemToRemove: dllistnode = item
          item = item.next

          neighbor: RclItem = _rclBaseList.remove(itemToRemove)
          chromosome[neighbor.index] = 0
          removedSet.add(neighbor.index)
          removed += 1

        else:
          item = item.next

    # Debug only
    for i in chromosome:
      assert i != None

    return chromosome

  # Generate the chromosomes
  return [createChromosome()] * population_size
