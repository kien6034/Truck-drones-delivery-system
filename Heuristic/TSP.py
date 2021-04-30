import numpy as np
import matplotlib.pyplot as plt
from Heuristic.Parameter import *
from Heuristic.Congestion import *
from Heuristic.Vehicles import *
import random

class TSP():
    def __init__(self, graph):
        self.graph = graph

        self.customers = set()
        self.uav_customers = set()

        self.d_nodes = set() #departure node
        self.a_nodes = set() #arrival node
        self.nodes_dict = dict()

        self.start_node = None
        self.end_node = None

        self.start_time = None

        #time matrix
        self.TL = None
        self.TU = None
        self.H = dict()

        #vehicle
        self.truck = None
        self.UAVs = set()

    
    def create_truck_and_uav(self, truck_num, uav_num):
        self.truck = Truck(1)

        i = 1
        while i <= uav_num:
            self.UAVs.add(UAV(i))
            i+=1

    def random_generate(self, num_of_customers, num_uav_customers):
        """ GENERATE customers"""
        i = 0

        #element need to take
        k = num_of_customers + 2
        num_uav_customers +=2
        for node in self.graph.nodes:
            if i == k:
                break
            
            self.nodes_dict[node] = i

            if i == 0:
                #get first node 
                self.start_node = node
                self.d_nodes.add(node)
                
            elif i == (k-1):
                self.end_node = node
                self.a_nodes.add(node)
            else:
                if i < num_uav_customers:
                    self.uav_customers.add(node)

                self.customers.add(node)

                self.d_nodes.add(node)
                self.a_nodes.add(node)
            
            i += 1
        
        """Generate others"""
        self.start_time = random.randint(0,24)

        #generate h 
        for i in range(24):
            self.H[i] = (i, i+1)
     

    
    def draw(self, EA = None):
        #draw tsp
        #start depot and end depot
        plt.plot(self.start_node.lon, self.start_node.lat, "ro", markersize = 12)
        plt.plot(self.end_node.lon, self.end_node.lat, 'ro', markersize = 9)

        #customer
        for c in self.customers:
            plt.plot(c.lon, c.lat, "bo", markersize = 12)
        
        #uav customer 
        for uc in self.uav_customers:
            plt.plot(uc.lon, uc.lat, "mo", markersize = 9)

        if EA:
            x_a = []
            y_a = []
            z_a = []
            for node in EA:
                if EA[node] != INFINITY:        
                    plt.annotate(round(EA[node], 2), (node.lon, node.lat), (node.lon + 10, node.lat +10), arrowprops= {"arrowstyle":"<-"})


    
    def label_setting(self, start_node, start_time):
        EA = {}

        S = self.graph.nodes.copy()
        #print(f"Label setting for {start_node.id} at time {start_time} is running")
        #init time arrival from start_node to other nodes 
        for node in S:
            if node == start_node:
                EA[node] = start_time
            else:
                EA[node] = INFINITY
        
        i = 0
        while S:
            
            #pick the best node with earliest time arrival
            best_node = None
            for node in S:
                if best_node == None:
                    best_node = node
                else:
                    if EA[node] < EA[best_node]:
                        best_node = node
            
            if EA[best_node] == INFINITY:
                S.remove(best_node)
                continue
                
            for neigbor in best_node.neigbors:
                EA[neigbor] = min(EA[neigbor], EA[best_node] + self.time_travel(EA[best_node], best_node, neigbor))

            S.remove(best_node)

            # print(f"Finish label setting for node {i}")
            # i +=1

        return EA

    def time_travel(self, start_time, node, neigbor):
        arrival_time = start_time

        #get arc
        try: 
            arc = self.graph.get_edge(node, neigbor)
        except:
            print("Arc is not found!")

        #get length
        l = arc.length
        speed_limit = arc.speed_limit

        #define time period 
        f = calculate_time_period(start_time % 24)

        #current start and end time period 
        ts = start_time % 24
        tf = TIME_PERIOD[f][1]

        #calculate time interval
        time_interval = tf - ts

        #calculate current congestion
        e = calculate_z1_congestion(f, arc)

        #time taken on the segment
        time_on_seg = l /( speed_limit * (1-e))

        while time_on_seg > time_interval:
            #update remain l
            l = l - speed_limit * (1-e) * time_interval 

            #update arrival time
            arrival_time += time_interval

            #move to new time period 
            if f < 3:
                f = f +1
            else:
                f = 0
            
            #calculate new time interval
            time_interval = TIME_PERIOD[f][1] - TIME_PERIOD[f][0]

            #recalculate time congestion 
            e = calculate_z1_congestion(f, arc)

            #calculate new time taken on the remain arc
            time_on_seg = 1 / (speed_limit * (1-e))
        
        #update arrival time if time_on_Seg < time_interavl
        arrival_time += time_on_seg

        

        return (arrival_time - start_time)
    

    def create_3d_time_matrix(self, isRun = False):

        if isRun:
            try:
                print("Loading time matrix data")
                self.TL = np.load("np_data/TL.npy")
                self.TU = np.load("np_data/TU.npy")
                print(self.TL.shape)
            except:
                print("Something went wrong with reading time matrix")
        else:
            print("creating 3d time matrix....")
           
            self.TL = np.empty(((len(self.d_nodes) + 1), (len(self.d_nodes) + 1), NUM_INTERVAL))
            self.TU = np.empty(((len(self.d_nodes) + 1), (len(self.d_nodes) + 1), NUM_INTERVAL))

            
            try: 
                for d_node in self.d_nodes:
                    
                    for h in self.H:
                        EAL = dict()
                        EAU = dict()

                        EAL = self.label_setting(d_node, self.H[h][0])
                        EAU = self.label_setting(d_node, self.H[h][1])
                        
                        #assign value for time matrix
                        for a_node in self.a_nodes:
                            self.TL[self.nodes_dict[d_node]][self.nodes_dict[a_node]][h] = EAL[(a_node)]
                            self.TU[self.nodes_dict[d_node]][self.nodes_dict[a_node]][h] = EAU[(a_node)]
                    print(f"Time matrix for node {self.nodes_dict[d_node]} is done!")
            
            except:
                print("Creating time matrix error!!!")

            np.save("np_data/TL.npy", self.TL)
            np.save("np_data/TU.npy", self.TU)
    
    def get_time_travel(self, d_node, a_node, h):
      
        return self.TL[self.nodes_dict[d_node]][self.nodes_dict[a_node]][h], self.TU[self.nodes_dict[d_node]][self.nodes_dict[a_node]][h]
      
