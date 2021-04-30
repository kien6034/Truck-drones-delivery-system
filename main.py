from Map.OsmHandler import *
from Map.Graph import *
from Heuristic.TSP import *
from Heuristic.APCS import *

graph = Graph()

print("Reading data....")
mRead = MapReader(graph)
mRead.apply_file("data/map7.osm")

graph.draw("road_type")

tsp = TSP(graph)
tsp.random_generate(15, 10)
tsp.create_truck_and_uav(1,4)
#EA = tsp.label_setting(tsp.start_node, tsp.start_time)
tsp.create_3d_time_matrix(isRun=False)
tsp.draw()

"""APCS"""
apcs = APCS(tsp)
apcs.create_ants()

apcs.run()


plt.show()