#!/usr/bin/env python3.10
# -*- coding: UTF-8 -*-

from typing import TextIO
from haversine import haversine


def read_elem(instance: TextIO) -> list[str]:
  with instance:
    return instance.read().split()


def read_input(instance: TextIO):
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

  coordenates = [None] * nb_nodes  # List of tuples
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

  return (distance_matrix, distance_from_base, communication_net)


def haversineCalculation(point1, point2):
  return haversine(point1, point2, 'm')


def compute_distance_matrix(qtd, points):
  distance_matrix = [[0 for i in range(int(qtd))] for j in range(int(qtd))
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
  nb_nodes = int(qtd)
  distance_from_base = [None] * nb_nodes

  for i in range(nb_nodes - 1):
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
  net_matrix = [[] for i in range(int(qtd))
                ]  #lista que guarda comunicacao dos nodes pra cada node.

  #nodesRelacionados = []
  for i in range(int(qtd) - 1):
    #nodesRelacionados.clear()
    for j in range(int(qtd) - 1):
      if i == j: continue
      try:
        dist = haversineCalculation(points[i], points[j])
        if dist < int(radius) * 2:
          #nodesRelacionados.append(j+1)
          net_matrix[i].append(j + 1)
      except:
        print('Nao conseguiu calcular a distancia entre: ', i + 1, ' e ', j + 1)
    print('Nó', i + 1, 'se comunica com:', net_matrix[i])

  print(net_matrix)
  return net_matrix
