#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

from brkga_mp_ipr.types import BaseChromosome
from typing import NamedTuple


class Item(NamedTuple):
  index: int
  gene: float


class Decoder:

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
