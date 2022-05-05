#!/usr/bin/env python3

# --- Imports
import csv
from ortools.constraint_solver import routing_enums_pb2, pywrapcp
from dataclasses import dataclass
import math

# --- Constants
LAT_SHIFT = -34.928585
LONG_SHIFT = 138.6000

# --- Classes
@dataclass
class Point():
    human: str
    lat: float
    long: float
    points: int

    @property
    def lat_shifted(self):
        return self.lat + LAT_SHIFT

    @property
    def long_shifted(self):
        return self.long + LONG_SHIFT

    def find_distance(self, __o: object) -> float:
        return math.dist([self.lat, self.long], [__o.lat, __o.long])

    def __repr__(self):
        return self.human

    def __eq__(self, __o: object) -> bool:
        return (self.lat == __o.lat) and (self.long == __o.long)

# --- Functions
def to_vector(inputing: float) -> int:
    return int(inputing * 10000)

def find_distances(initial: Point, others: list) -> list:
    """Get the distances of every point to this point and return in a list

        Changes to vectors because why not
    Args:
        initial (Point): Point to compare others too.
        others (list): Other points.

    Returns:
        list: list of distances
    """

    data = []
    for other in others:
        data.append(to_vector(initial.find_distance(other)))

    return data


def print_solution(manager, routing, solution):
    print(f'Objective: {solution.ObjectiveValue()}')

    index = routing.Start(0)
    
    plan_output = f'Route:\n'
    route_distance = 0

    indicies = []

    while not routing.IsEnd(index):
        indicies.append(manager.IndexToNode(index))
        plan_output += f'{manager.IndexToNode(index)} -> '
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    
    indicies.append(manager.IndexToNode(index))
    plan_output += f' {manager.IndexToNode(index)}'
    print(f'{plan_output}\nRoute Distance: {route_distance}')
    return indicies

if __name__ == '__main__':
    with open('./calculated_lat_long.csv', 'r') as csv_file:
        location_reader = csv.DictReader(csv_file, delimiter=',')

        points = []

        for loc in location_reader:
            points.append(Point(
                loc['Location'],
                float(loc['Lat']),
                float(loc['Long']),
                float(loc['Points'])
            ))

    data_model = []

    for i, point in enumerate(points):
        data_model.append(find_distances(point, points))
    
    # Routing index manager?
    manager = pywrapcp.RoutingIndexManager(len(data_model), 1, 0)

    # Routing model?
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(fr_in, to_in):
        from_node = manager.IndexToNode(fr_in)
        to_node = manager.IndexToNode(to_in)
        return data_model[from_node][to_node]


    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Cost of arc?
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # First solution?
    search_params = pywrapcp.DefaultRoutingSearchParameters()
    search_params.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    solution = routing.SolveWithParameters(search_parameters=search_params)

    if solution:
        ir = print_solution(manager, routing, solution)

    for x in ir:
        print(points[x].human)
