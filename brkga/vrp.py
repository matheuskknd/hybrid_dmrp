
# import localsolver
from haversine import haversine
import sys

def read_elem(filename):
    with open(filename) as f:
        return [str(elem) for elem in f.read().split()]


def main(instance_file, str_nb_drones):
    nb_trucks = int(str_nb_drones)
   
    (all_distances, base_distance, comunicacoes) = read_input_cvrp(instance_file)
    print()
    if nb_trucks == 0:
        nb_trucks = 1

    # with localsolver.LocalSolver() as ls:
    #     model = ls.model
    #     decisoes = [model.bool() for i in range(len(base_distance)-1)]

    #     # Create demands as an array to be able to access it with an "at" operator
    #     demands_array = model.array(demands)

    # #     # Create distance as an array to be able to acces it with an "at" operator
    #     distance_array = model.array()
    #     for n in range(all_distances):
    #         distance_array.add_operand(model.array(all_distances[n]))

        # distance_base_array = model.array(base_distance)

    #     route_distances = [None] * nb_trucks

    # #     # A truck is used if it visits at least one customer
    # #     trucks_used = [(model.count(customers_sequences[k]) > 0) for k in range(nb_trucks)]
    # #     nb_trucks_used = model.sum(trucks_used)
    #     nb_trucks_used = 1
    #     # for k in range(nb_trucks):
    # #         sequence = customers_sequences[k]
    # #         c = model.count(sequence)

    # #         # Quantity in each truck
    # #         demand_selector = model.lambda_function(lambda i: demands_array[sequence[i]])
    # #         route_quantity = model.sum(model.range(0, c), demand_selector)
    # #         model.constraint(route_quantity <= truck_capacity)

    # #         # Distance traveled by each truck
    # #         dist_selector = model.lambda_function(lambda i: model.at(distance_array, sequence[i - 1], sequence[i]))
    # #         route_distances[k] = model.sum(model.range(1, c), dist_selector) + \
    # #                              model.iif(c > 0, distance_base_array[sequence[0]] + distance_base_array[sequence[c - 1]], 0)

    #     # Total distance traveled
    #     total_distance = model.sum(route_distances)

    #     # Objective: minimize the number of trucks used, then minimize the distance traveled
    # #     model.minimize(nb_trucks_used)
    #     model.minimize(total_distance)

        # model.close()

        # ls.param.time_limit = 20

        # ls.solve()

        #
        # Writes the solution in a file with the following format:
        #  - number of trucks used and total distance
        #  - for each truck the nodes visited (omitting the start/end at the depot)
        #
    #     if len(sys.argv) >= 3:
    #         with open(sol_file, 'w') as f:
    #             f.write("%d %d\n" % (nb_trucks_used.value, total_distance.value))
    #             for k in range(nb_trucks):
    #                 if trucks_used[k].value != 1: continue
    #                 # Values in sequence are in [0..nbCustomers-1]. +2 is to put it back in [2..nbCustomers+1]
    #                 # as in the data files (1 being the depot)
    #                 for customer in customers_sequences[k].value:
    #                     f.write("%d " % (customer + 2))
    #                 f.write("\n")



def read_input_cvrp(filename):
    file_it = iter(read_elem(sys.argv[1]))
    
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
    
    while(nb_nodes-1 > 0):
        x_coordenates = next(file_it)
        y_coordenates = next(file_it)
        str_communicationRadius = next(file_it)

        coordenates[nb_nodes-2] = (float(x_coordenates), float(y_coordenates))
        print('Coordenada do nó', nb_nodes, '=', coordenates[nb_nodes-2])        
        nb_nodes = nb_nodes-1
    
    print('\n[OBS: Todas as medidas de distância utilizadas neste programa estão em metros.]')
    distance_matrix = compute_distance_matrix(str_nb_nodes, coordenates) #matriz com a dist calculada entre as coordenadas de cada sensor
    distance_from_base = compute_distance_from_base(str_nb_nodes, coordenates, base) #lista com a dist calculada entre a base e cada node
    communication_net = compute_communication_net(str_communicationRadius, coordenates) #Lista de elementos dentro do raio de comunicação

    return (distance_matrix, distance_from_base, communication_net)


def haversineCalculation(point1, point2):
    return haversine(point1, point2, 'm')

def compute_distance_matrix(qtd, points):    
    distance_matrix = [[0 for i in range(int(qtd))] for j in range(int(qtd))] #matriz quadrada de tam igual ao num de nodes.
    
    #TEMPLATE => i = coordenadaX (latitude), j = coordenadasY (longitude)

    for i in range(int(qtd)-1):
        distance_matrix[i][i] = 0
        for j in range(int(qtd)-1):
            if distance_matrix[i][j] != 0: continue  # volta = ida, calculo realizado 1 vez.
            if i == j: 
                distance_matrix[i][j] = 0   # mesmo pto, dist = 0.
                continue
            
            try:
                dist = haversineCalculation(points[i], points[j])
                distance_matrix[i][j] = dist
                distance_matrix[j][i] = dist
                print('Distancia entre nó', i, 'e nó', j, ':', dist)
            except:
                print('Nao conseguiu calcular a distancia entre: ', i, ' e ',j)
                continue
    print()
    # print(distance_matrix)
    return distance_matrix


def compute_distance_from_base(qtd, points, base):
    nb_nodes = int(qtd)
    distance_from_base = [None] * nb_nodes

    for i in range(nb_nodes-1):
        try:
            dist = haversineCalculation(base, points[i])
            distance_from_base[i] = dist
            print('Distancia entre a Base e o nó ', i, ': ', dist)
        except:
            print('Nao conseguiu calcular a distancia entre a base e', points[i])
            continue
    
    print()
    # print(distance_from_base)
    return distance_from_base

def compute_communication_net(radius, points):
    net_matrix = [[] for i in range(40)] #lista que guarda comunicacao dos nodes pra cada node.
    
    nodesRelacionados = []
    for i in range(40):
        nodesRelacionados.clear()
        for j in range(40):
            if i==j: continue
            try:
                dist = haversineCalculation(points[i], points[j])
                if dist < int(radius)*2:
                    nodesRelacionados.append(j) 
                    # net_matrix[i].append(points[j])
            except:
                print('Nao conseguiu calcular a distancia entre: ', i, ' e ',j)               
        
        print('Nó', i, 'se comunica com:', nodesRelacionados)
        net_matrix[i] = nodesRelacionados

    # print(net_matrix)
    return net_matrix


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python pcvr.py input_file [output_file] [time_limit] [nb_trucks]")
        sys.exit(1)

    instance_file = sys.argv[1]
    print("Utilizando arquivo de entrada: ", instance_file)
    print()
    main(instance_file, "1")
