#!/usr/bin/env python3.10
# -*- coding: UTF-8 -*-

from .ConstructiveHeurisitc import (RclItem, generateRclBase)
from zipfile import (ZipFile, ZIP_DEFLATED, is_zipfile)
from os.path import (basename, dirname, join)
from typing import (Any, Optional, TextIO)
from haversine import haversine
from os import getcwd
import timeit
import json

assert dirname(dirname(__file__)) == getcwd(), "Invalid working directory!"


class InstanceData(object):

  _CACHE_PARAMS: dict[str, Any] = {
    "file": join(dirname(dirname(__file__)), "instances", ".cached.zip"),
    "compression": ZIP_DEFLATED,
    "compresslevel": 9,
  }

  def isCached(instanceName: str) -> bool:
    if not is_zipfile(InstanceData._CACHE_PARAMS["file"]):
      return False

    with ZipFile(mode="r", **InstanceData._CACHE_PARAMS) as archive:
      return basename(instanceName) in frozenset(archive.namelist())

  def loadCached(instanceName: str) -> "InstanceData":

    def decoder(_vars: dict[str, Any]) -> InstanceData:
      return InstanceData(
        _vars["distance_matrix"],
        _vars["distance_from_base"],
        _vars["communication_net"],
        tuple(_vars["base"]),
        [(i, j) for (i, j) in _vars["coordenates"]],
        [RclItem(i, c) for (i, c) in _vars["centralities"]],
        None,
      )

    # Load cached
    with ZipFile(mode="r", **InstanceData._CACHE_PARAMS) as archive:
      with archive.open(basename(instanceName), "r") as cacheFile:
        return json.loads(cacheFile.read().decode("UTF-8"), object_hook=decoder)

  def __init__(
    self,
    distance_matrix: list[list[float]],
    distance_from_base: list[float],
    communication_net: list[list[int]],
    base: tuple[float, float],
    coordenates: list[tuple[float, float]],
    centralities: list[RclItem],
    instanceName: Optional[str],
  ) -> None:

    self.base: tuple[float, float] = base
    """The base location."""

    self.coordenates: list[tuple[float, float]] = coordenates
    """Each node location (excluding the base)."""

    self.distance_matrix: list[list[float]] = distance_matrix
    """Square matrix with all distance between vertices."""

    self.distance_from_base: list[float] = distance_from_base
    """Array of distances from the base for each vertex."""

    self.communication_net: list[list[int]] = communication_net
    """Sparse matrix containing the reach graph."""

    self.centralities: list[RclItem] = centralities
    """Array containing all non-base vertices index/ID and respective eigenvector centrality value."""

    # Loaded cached
    if instanceName == None:
      return

    # Save cached
    with ZipFile(mode="a", **InstanceData._CACHE_PARAMS) as archive:
      with archive.open(basename(instanceName), "w") as cacheFile:
        asDict: dict[str, Any] = vars(self)
        cacheFile.write(json.dumps(asDict, ensure_ascii=False).encode("UTF-8"))


def read_elem(instance: TextIO) -> list[str]:
  with instance:
    return instance.read().split()


def read_input(instance: TextIO) -> InstanceData:
  startTime: float = timeit.default_timer()

  if InstanceData.isCached(instance.name):
    print(f"Loading cached: {instance.name} ... ", end="")
    instanceData: InstanceData = InstanceData.loadCached(instance.name)
    print(f"Done after:  {timeit.default_timer() - startTime:.2f} s")
    return instanceData

  file_it = iter(read_elem(instance))

  str_nb_nodes = next(file_it)
  nb_nodes = int(str_nb_nodes)
  print("Number of nodes: " + str_nb_nodes)

  x_base = next(file_it)
  y_base = next(file_it)
  base = (float(x_base), float(y_base))
  print("Base: ", base)
  print()
  next(file_it)

  coordenates = [None] * (nb_nodes-1)  # List of tuples
  for i in range(nb_nodes - 1):
    x_coordenates = next(file_it)
    y_coordenates = next(file_it)
    str_communicationRadius = next(file_it)

    coordenates[i] = (float(x_coordenates), float(y_coordenates))
    print('Coordenada do nó', i + 1, '=', coordenates[i])

  print('\n[OBS: Todas as medidas de distância utilizadas neste programa estão em metros.]')
  distance_matrix = compute_distance_matrix(str_nb_nodes,
                                            coordenates)  #matriz com a dist calculada entre as coordenadas de cada sensor
  distance_from_base = compute_distance_from_base(str_nb_nodes, coordenates,
                                                  base)  #lista com a dist calculada entre a base e cada node
  communication_net = compute_communication_net(str_nb_nodes,
                                                str_communicationRadius,
                                                coordenates)  #Lista de elementos dentro do raio de comunicação

  instanceData: InstanceData = InstanceData(
    distance_matrix,
    distance_from_base,
    communication_net,
    base,
    coordenates,
    centralities=generateRclBase(distance_matrix, communication_net),
    instanceName=instance.name,
  )

  print(f"Instance fully read after:  {timeit.default_timer() - startTime:.2f} s")
  return instanceData


def haversineCalculation(point1, point2):
  return haversine(point1, point2, 'm')


def compute_distance_matrix(qtd, points):
  distance_matrix = [[0
                      for i in range(int(qtd) - 1)]
                     for j in range(int(qtd) - 1)
                     ]  #matriz quadrada de tam igual ao num de nodes.

  #TEMPLATE => i = coordenadaX (latitude), j = coordenadasY (longitude)

  for i in range(int(qtd) - 1):
    distance_matrix[i][i] = 0
    for j in range(int(qtd) - 1):
      if distance_matrix[i][j] != 0:
        continue  # volta = ida, calculo realizado 1 vez.
      if i == j:
        distance_matrix[i][j] = 0  # mesmo pto, dist = 0.
        continue

      try:
        dist = haversineCalculation(points[i], points[j])
        distance_matrix[i][j] = dist
        distance_matrix[j][i] = dist
        # print('Distancia entre nó', i+1, 'e nó', j+1, ':', dist)
      except:
        print('Nao conseguiu calcular a distancia entre: ', i + 1, ' e ', j + 1)
        continue
  # print()
  # print(distance_matrix)
  return distance_matrix


def compute_distance_from_base(qtd, points, base):
  nb_nodes = int(qtd) - 1
  distance_from_base = [None] * nb_nodes

  for i in range(nb_nodes):
    try:
      dist = haversineCalculation(base, points[i])
      distance_from_base[i] = dist
      # print('Distancia entre a Base e o nó ', i+1, ': ', dist)
    except:
      print('Nao conseguiu calcular a distancia entre a base e', points[i])
      continue

  print()
  # print(distance_from_base)
  return distance_from_base


def compute_communication_net(qtd, radius, points):
  net_matrix = [[] for i in range(int(qtd) - 1)
                ]  #lista que guarda comunicacao dos nodes pra cada node.

  #nodesRelacionados = []
  for i in range(int(qtd) - 1):
    id = i + 1
    #nodesRelacionados.clear()
    for j in range(int(qtd) - 1):
      id2 = j + 1
      if i == j: continue
      try:
        dist = haversineCalculation(points[i], points[j])
        if dist < int(radius) * 2:
          #nodesRelacionados.append(j+1)
          net_matrix[i].append(j)
      except:
        print('Nao conseguiu calcular a distancia entre: ', id, ' e ', id2)
    print('Nó', id, 'se comunica com:', net_matrix[i])

  print(net_matrix)
  return net_matrix
