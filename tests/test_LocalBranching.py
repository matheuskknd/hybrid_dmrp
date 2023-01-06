#!/usr/bin/env python3.10
# -*- coding: UTF-8 -*-

from docplex.mp.engine import JobSolveStatus
from .Testing import ExtendedTestCase
from networkx import MultiDiGraph
from typing import Any

from hybrid_dmrp.MinimumDominatingSet import HybridBrkgaSolution
from hybrid_dmrp.LocalBranching import (
  LocalBranchingSolution,
  solveLocalBranching,
)


class MockedHBrkgaSolution(HybridBrkgaSolution):

  def __init__(self, **kwargs: dict[str, Any]) -> None:
    super().__init__(vrpCost=0.0, **kwargs)


class TestLocalBranching(ExtendedTestCase):

  def _checkSolution(self, graph: MultiDiGraph, mds: set[int]) -> float:
    mds = set(map(lambda i: i + 1, mds))

    totalCost: float = 0.0
    visited: set[int] = set()

    for n in mds:
      if n in visited or not graph.has_edge(0, n):
        continue

      totalCost += graph[0][n][0]["cost"]
      i: int = n

      while i != 0:
        inEdges: list[tuple[int, int]] = list(graph.in_edges(i, data=False))

        outEdges: list[tuple[int, int,
                             dict]] = list(graph.out_edges(i, data=True))

        # Check how many edges come to and leave from i
        self.assertEqual(1, len(outEdges),
                         f"Node {i} dispatches more than one vehicle.")

        self.assertEqual(1, len(inEdges),
                         f"Node {i} receives more than one vehicle.")

        # Check if i was not visited before
        self.assertFalse(i in visited, f"Node {i} visted more than once.")

        # Save i as visited, and follow the directed graph
        totalCost += outEdges[0][2]["cost"]
        visited.add(i)

        i = outEdges[0][1]

    # Assert the graph full coverage
    if graph.number_of_nodes() != 0:
      self.assertEqual(visited, mds,
                       f"Unreachable nodes: {mds.difference(visited)}")

    return totalCost

  # Asserts the optimal solution is found for some trivial instance
  # Autonomy: 10k
  # |N| = 1
  # |V| = 1
  #
  # Optima: (0,1) -> (1,0)
  def test_solveLocalBranching_trivial_n1_001(self) -> None:
    mockedHBrkagaSol: MockedHBrkgaSolution = MockedHBrkgaSolution(
      minimumDominatingSet={0},
      autonomy=10_000,
    )

    # Call target method
    print("\n########\n")
    babSolution: LocalBranchingSolution = solveLocalBranching(
      mockedHBrkagaSol,
      seed=1,
      allDistances=[0],
      baseDistance=[10],
      isCLI=False,
      quiet=False,
    )

    # Assert the result
    self.assertEqual(babSolution.babstatus, JobSolveStatus.OPTIMAL_SOLUTION)
    self.assertEqual(babSolution.babGraph.number_of_nodes(), 2)
    self.assertEqual(babSolution.babGraph.number_of_edges(), 2)
    self.assertEqual(babSolution.babCost, 20)
    self.assertEqual(
      babSolution.babCost,
      self._checkSolution(
        babSolution.babGraph,
        babSolution.minimumDominatingSet,
      ))

  # Asserts the optimal solution is found for some trivial instance
  # Autonomy: 10k
  # |N| = 2
  # |V| = 2
  #
  # Optima: (0,1) -> (1,2) -> (2,0)
  def test_solveLocalBranching_trivial_n2_001(self) -> None:
    mockedHBrkagaSol: MockedHBrkgaSolution = MockedHBrkgaSolution(
      minimumDominatingSet={0, 1},
      autonomy=10_000,
    )

    # Call target method
    print("\n########\n")
    babSolution: LocalBranchingSolution = solveLocalBranching(
      mockedHBrkagaSol,
      seed=1,
      allDistances=[[0, 1], [1, 0]],
      baseDistance=[10, 10],
      isCLI=False,
      quiet=False,
    )

    # Assert the result
    self.assertEqual(babSolution.babstatus, JobSolveStatus.OPTIMAL_SOLUTION)
    self.assertEqual(babSolution.babGraph.number_of_nodes(), 3)
    self.assertEqual(babSolution.babGraph.number_of_edges(), 3)
    self.assertEqual(babSolution.babCost, 21)
    self.assertEqual(
      babSolution.babCost,
      self._checkSolution(
        babSolution.babGraph,
        babSolution.minimumDominatingSet,
      ))

  # Asserts the optimal solution is found for some trivial instance
  # Autonomy: 10k
  # |N| = 2
  # |V| = 2
  #
  # Optima: (0,1) -> (1,0) | (0,2) -> (2,0)
  def test_solveLocalBranching_trivial_n2_002(self) -> None:
    mockedHBrkagaSol: MockedHBrkgaSolution = MockedHBrkgaSolution(
      minimumDominatingSet={0, 1},
      autonomy=10_000,
    )

    # Call target method
    print("\n########\n")
    babSolution: LocalBranchingSolution = solveLocalBranching(
      mockedHBrkagaSol,
      seed=1,
      allDistances=[[0, 10], [10, 0]],
      baseDistance=[1, 1],
      isCLI=False,
      quiet=False,
    )

    # Assert the result
    self.assertEqual(babSolution.babstatus, JobSolveStatus.OPTIMAL_SOLUTION)
    self.assertEqual(babSolution.babGraph.number_of_nodes(), 3)
    self.assertEqual(babSolution.babGraph.number_of_edges(), 4)
    self.assertEqual(babSolution.babCost, 4)
    self.assertEqual(
      babSolution.babCost,
      self._checkSolution(
        babSolution.babGraph,
        babSolution.minimumDominatingSet,
      ))

  # Asserts the optimal solution is found for some trivial instance
  # Autonomy: 20
  # |N| = 2
  # |V| = 2
  #
  # Optima: (0,1) -> (1,0) | (0,2) -> (2,0)
  def test_solveLocalBranching_trivial_n2_003(self) -> None:
    mockedHBrkagaSol: MockedHBrkgaSolution = MockedHBrkgaSolution(
      minimumDominatingSet={0, 1},
      autonomy=20,
    )

    # Call target method
    print("\n########\n")
    babSolution: LocalBranchingSolution = solveLocalBranching(
      mockedHBrkagaSol,
      seed=1,
      allDistances=[[0, 1], [1, 0]],
      baseDistance=[10, 10],
      isCLI=False,
      quiet=False,
    )

    # Assert the result
    self.assertEqual(babSolution.babstatus, JobSolveStatus.OPTIMAL_SOLUTION)
    self.assertEqual(babSolution.babGraph.number_of_nodes(), 3)
    self.assertEqual(babSolution.babGraph.number_of_edges(), 4)
    self.assertEqual(babSolution.babCost, 40)
    self.assertEqual(
      babSolution.babCost,
      self._checkSolution(
        babSolution.babGraph,
        babSolution.minimumDominatingSet,
      ))

  # Asserts the optimal solution is found for some trivial instance
  # Autonomy: 20
  # |N| = 2
  # |V| = 2
  #
  # Optima:
  def test_solveLocalBranching_trivial_n4_001(self) -> None:
    mockedHBrkagaSol: MockedHBrkgaSolution = MockedHBrkgaSolution(
      minimumDominatingSet={0, 1, 2},
      autonomy=10_000,
    )

    # Call target method
    print("\n########\n")
    babSolution: LocalBranchingSolution = solveLocalBranching(
      mockedHBrkagaSol,
      seed=1,
      allDistances=[[0, 3, 3], [3, 0, 3], [3, 3, 0]],
      baseDistance=[10, 10, 10],
      isCLI=False,
      quiet=False,
    )

    # Assert the result
    self.assertEqual(babSolution.babstatus, JobSolveStatus.OPTIMAL_SOLUTION)
    self.assertEqual(babSolution.babGraph.number_of_nodes(), 4)
    self.assertEqual(babSolution.babGraph.number_of_edges(), 4)
    self.assertEqual(babSolution.babCost, 26)
    self.assertEqual(
      babSolution.babCost,
      self._checkSolution(
        babSolution.babGraph,
        babSolution.minimumDominatingSet,
      ))

  # Asserts the solution is found to be infeasible for some trivial instance
  # Autonomy: 0
  # |N| = 1
  # |V| = 1
  #
  # Optima: INFEASIBLE
  def test_solveLocalBranching_infeasible_n1_001(self) -> None:
    mockedHBrkagaSol: MockedHBrkgaSolution = MockedHBrkgaSolution(
      minimumDominatingSet={0},
      autonomy=0,
    )

    # Call target method
    print("\n########\n")
    babSolution: LocalBranchingSolution = solveLocalBranching(
      mockedHBrkagaSol,
      seed=1,
      allDistances=[[0]],
      baseDistance=[10],
      isCLI=False,
      quiet=False,
    )

    # Assert the result
    self.assertEqual(babSolution.babstatus, JobSolveStatus.INFEASIBLE_SOLUTION)
    self.assertEqual(babSolution.babGraph.number_of_nodes(), 0)
    self.assertEqual(babSolution.babGraph.number_of_edges(), 0)
    self.assertEqual(babSolution.babCost, float("inf"))
    self.assertEqual(
      0,
      self._checkSolution(
        babSolution.babGraph,
        babSolution.minimumDominatingSet,
      ))
