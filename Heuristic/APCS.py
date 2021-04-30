from Heuristic.Parameter import *
from Heuristic.Vehicles import *
import math
import matplotlib.pyplot as plt
from numpy.random import choice

class APCS():
    def __init__(self, tsp):
        self.uav_p = dict()
        self.truck_p = dict()
        self.tsp = tsp
        self.bestO = INFINITY
        
        self.ants = set()    

    def create_ants(self, number=MAX_ANTPAIR):
        for i in range(number):
            ant = Ant(self.tsp)
            self.ants.add(ant)

    def init_pheromone(self):
        #TRUCK PHEROMONE
        for nodei in self.tsp.d_nodes:
            for nodej in self.tsp.a_nodes:
                if nodei != nodej:
                    self.truck_p[(nodei, nodej)] = MIN_TRUCK_PHEROMONE
        
        #UAV PHEROMONE
        for uav in self.tsp.UAVs:
            for nodei in self.tsp.d_nodes:
                for nodev in self.tsp.uav_customers:
                    if nodei != nodev:
                        self.uav_p[(uav, nodei, nodev)] = MIN_UAV_PHEROMONE

    def run(self):
        self.init_pheromone()
        for i in range(MAX_ITERATION):
            #set iter objective
            iterO = INFINITY

            #init pheromone copy
            uav_p = self.uav_p.copy()
            truck_p = self.truck_p.copy()

            for ant in self.ants:
                #get alpha, beta
                alpha = min(5, 0.04 * (i+1))
                beta = min(5, 0.04 * (i+1))

                TSPtour, UAVcustInfo, node_and_uavs_to_lauches = ant.construct_solution(truck_p, uav_p, alpha, beta)
                ant.update_local_pheromone(truck_p, uav_p, TSPtour, UAVcustInfo)
                plt.show()
                objVal, activityTimings = ant.build_approx_schedule(TSPtour, UAVcustInfo, node_and_uavs_to_lauches)
                print(f"Ant finished !")
                break
            break


class Ant():
    def __init__(self, tsp):
        """
            Info needed: 
                - uav pheromonem
                - truck pheromone
        """
        self.tsp = tsp

    def construct_solution(self, truck_p, uav_p, alpha, beta):
        #init truck route with depot 0
        tsp_tour = [self.tsp.start_node]

        #init uavCustInfo
        uav_cust_info = list()

        #info about which uav start from which node and its target customer 
        node_and_uavs_to_lauches = dict()

        #get customer and uav customer set
        C = self.tsp.customers.copy()
        U = self.tsp.uav_customers.copy()
     
        time = CURRENT_TIME
        while C:
            """=============== First ant ================"""
            #get the last node from tsp tour
            current_node = tsp_tour[-1]

            #get time interval
            t = time % 24
            h = math.floor(t)

            all_nodes = dict()

            truck_candidates = list()
            truck_candidate_times = list()
            k = 0 # k to get num of truck candidates 
            #======== SUB_TRUCK_1: get truck candidate
            for node in C:
                #calculate time travel from current node to other node in c
                tl, tu = self.tsp.get_time_travel(current_node, node, h)
                time_travel = tl + (t - h) * (tu - tl) / 1 #(66) 
                
                all_nodes[node] = time_travel

                #get NUM_OF_CANDIDATES smallest element 
                if k < NUM_OF_CANDIDATES:
                    truck_candidates.append(node)
                    truck_candidate_times.append(time_travel) 
                
                else:
                    max_time_travel = 0
                    max_index = -1
                    for i in range(len(truck_candidates)):
                        if truck_candidate_times[i] > max_time_travel:
                            max_index = i
                            max_time_travel = truck_candidate_times[i]
                    
                    #compare max element againts cur time travel
                    if max_time_travel > time_travel:
                        truck_candidates[max_index] = node
                        truck_candidate_times[max_index] = time_travel
                k += 1

            #========== SUB_TRUCK_2: Calculate prob and select the next truck node
            total = 0
            truck_candidate_weights = list()
            for i in range(len(truck_candidates)):
                prob = math.pow(truck_p[(current_node, truck_candidates[i])], alpha) * math.pow(1/truck_candidate_times[i], beta)
                truck_candidate_weights.append(prob)

                total += prob
            
            for i in range(len(truck_candidates)):
                truck_candidate_weights[i] /= total
            
            #random choice
            bti = choice(range(len(truck_candidates)), p = truck_candidate_weights)
            next_truck_node = truck_candidates[bti]
            next_travel_time = truck_candidate_times[bti] - time

            #========= SUB_TRUCK_3
            #update time taken for truck operation
            time += next_travel_time
            time += self.tsp.truck.delivery_time

            #append next truck node to tsp tour
            tsp_tour.append(next_truck_node)

            #remove next truck node from C
            C.remove(next_truck_node)

            #remove next truck node in UAVcustInfo
            if next_truck_node in U:
                U.remove(next_truck_node)

            node_and_uavs_to_lauches[current_node] = list()

            """ ================= SECOND ANT =================="""
            for uav in self.tsp.UAVs:
                uav_candidate_nodes = list()
                uav_candidate_times = list()

                #=========SUB_UAV1: get candidate list for uav 
                k = 0
                for node in U: #uav customer nodes
                    
                    time_travel = uav.calculate_flight_time(current_node, node) + uav.delivery_time + uav.calculate_flight_time(node, next_truck_node)

                    if k < NUM_OF_UAV_CANDIDATES:
                        uav_candidate_nodes.append(node)
                        uav_candidate_times.append(time_travel)
                    else:
                        max_index = -1
                        max_time_travel = 0

                        for index in range(len(uav_candidate_nodes)):
                            if uav_candidate_times[index] > max_time_travel:
                                max_index = index
                                max_time_travel = uav_candidate_times[index]
                        
                        if max_time_travel > time_travel:
                            uav_candidate_nodes[max_index] = node
                            uav_candidate_times[max_index] = time_travel
                    k +=1
               
                #===========SUB_UAV2: Calculate prob  
                if len(uav_candidate_nodes) == 0:
                    continue
                    
                uav_candidate_weights = list()
                
                total = 0
                for i in range(len(uav_candidate_nodes)):
                    weight = math.pow(uav_p[(uav, current_node, uav_candidate_nodes[i])], alpha) * math.pow(1/uav_candidate_times[i], beta)
                    total += weight
                    uav_candidate_weights.append(weight)
                
                for i in range(len(uav_candidate_nodes)):
                    uav_candidate_weights[i] /= total
                
                #get next uav node
                next_uav_index = choice(range(len(uav_candidate_nodes)))
                next_uav_node = uav_candidate_nodes[next_uav_index]
                next_uav_times = uav_candidate_times[next_uav_index]
                
                #=============SUB_UAV3
                #TODO: check uav travel time vs truck travel time 

                #update time taken by uav
                time += self.tsp.truck.launch_time
                time += self.tsp.truck.retrival_time

                #append info to uav cust info
                uav_cust_info.append((uav, current_node, next_uav_node))

                #remove customer node from u and c
                try:
                    U.remove(next_uav_node)
                    C.remove(next_uav_node)
                except:
                    print("remove from U and C error!!")

                #append to node and uav to launches dict
                data =  {
                    'uav': uav,
                    'target': next_uav_node,
                    'next_truck_node': next_truck_node,
                    'travel_time': next_uav_times
                }
                node_and_uavs_to_lauches[current_node].append(data)

        #when C is empty
        tsp_tour.append(self.tsp.end_node)
        node_and_uavs_to_lauches[self.tsp.end_node] = {}
       
        return tsp_tour, uav_cust_info, node_and_uavs_to_lauches



    def update_local_pheromone(self, truck_p, uav_p, TSPtour, UAVcustInfo):
        #enhance divisification as the subsequent ant-pairs are not likely to travel in these arcs due to low pheromnone level
        pass
        for i in range(len(TSPtour)):
            if i > 0:
                truck_p[(TSPtour[i-1], TSPtour[i])] = truck_p[(TSPtour[i-1], TSPtour[i])] * (1-LOCAL_EVAPORATION_RATE)

        for custInfo in UAVcustInfo:
            uav_p[custInfo] = uav_p[custInfo] * (1- LOCAL_EVAPORATION_RATE)
           
       

    def build_approx_schedule(self, TSPtour, UAVcustInfo, node_and_uavs_to_lauches):
        objVal = 0
        activityTimings = dict()
        time = CURRENT_TIME

        truck = self.tsp.truck
        for node in TSPtour:
            activityTimings[node] = {
                'truck_arrival': None,
                'retrival': list(), 
                'delivery': None,
                'launch': list(),
                'truck_departure': None,     
            }
        for i, node in enumerate(TSPtour):
            if i == 0: 
                #define truck arrival time and  departure time  
                activityTimings[node]['truck_arrival'] = CURRENT_TIME

                #define order of lauches for 
                uavs_to_launch = node_and_uavs_to_lauches[node]
                uavs_to_launch = self.launch_order_cri1(uavs_to_launch)
                
                for element in uavs_to_launch:
                    data = {
                        'uav': element['uav'],
                        'start_time': time,
                        'completion_time': time + truck.launch_time,
                        'travel_time': element['travel_time']
                    }
                    activityTimings[node]['launch'].append(data) 
                    time += truck.launch_time

                #truck departure time
                activityTimings[node]['truck_departure'] = time
              

            elif i == len(TSPtour):
                pass
            else:
                preNode = TSPtour[i-1]
                #truck arrival time
                activityTimings[node]['truck_arrival'] = self.equation_66(preNode, node, activityTimings[preNode]['truck_departure'])

                #uav arrival
                uavs_to_retrieve = list()
                for element in activityTimings[preNode]['launch']:
                    data = {
                        'uav': element['uav'],
                        'arrival_time': element['completion_time'] + element['travel_time']
                    }
                    uavs_to_retrieve.append(data)


                uavs_to_retrieve.sort(key =lambda x: x['arrival_time']) #sort uav to retrive in order of increasing arrival time 
                activityTimings[node]['retrival'] = uavs_to_retrieve

                #order of launch 
                uavs_to_launch = node_and_uavs_to_lauches[node]
                uavs_to_launch = self.launch_order_cri1(uavs_to_launch)

                #get truck arrival time 
                truck_arrival_time = activityTimings[node]['truck_arrival']

                #modify order
                order_queue, queue_completion_time = self.modify_order(uavs_to_launch, uavs_to_retrieve, truck_arrival_time)

              
                activityTimings[node]['truck_departure'] = queue_completion_time 

                
                for element in order_queue:
                    if element["action"] == "launch":
                        data = {
                            'uav': element['element'],
                            'start_time': element['completion_time'] - truck.launch_time,
                            'completion_time': element['completion_time'],
                            'travel_time': element['travel_time']
                        }
                        activityTimings[node]['launch'].append(data)

        return objVal, activityTimings
    

    def launch_order_cri1(self, uavs_to_launch):
        """Define order of launch for UAV in order of decreasing total flight time"""
        uavs_to_launch.sort(key =lambda x: x['travel_time'], reverse=True)

        return uavs_to_launch


    def equation_66(self, current_node, next_node, time):
        t = time % 24
        h = math.floor(t)

        tl, tu = self.tsp.get_time_travel(current_node, next_node, h)
        time_travel = tl + (t - h) * (tu - tl) / 1 #(66) 

        return time_travel
    
    def modify_order(self, uavs_to_launch, uavs_to_retrieve, truck_arrival_time):
        order_queue = list()    
        
        truck = self.tsp.truck
        #=============SUB1: init order queue - Retrieve --> delivery --> launch
        
        for element in uavs_to_retrieve:
            data = {
                'action': 'retrieve',
                'element': element['uav'],
                'arrival_time': element['arrival_time'],
                'completion_time': None
            }
            order_queue.append(data)
        
        order_queue.append({'action': 'delivery', 'element': truck, 'completion_time': None})

        for element in uavs_to_launch:
            data = {
                'action': 'launch',
                'element': element['uav'],
                'completion_time': None,
                'travel_time': element['travel_time']
            }
            order_queue.append(data)
      
        #===============SUB2: shuffle
        retrieved_uavs = set()
        waiting_time = 0
        completion_time = truck_arrival_time

        for index, element in enumerate(order_queue):
            if element['action'] != 'retrieve':
                # if last elemetn 
                if index == len(order_queue) - 1:
                    completion_time += truck.launch_time
                    element['completion_time'] = completion_time

                    continue
                else:
                    continue

            #calculate wating time 
            waiting_time = element['arrival_time'] - completion_time

            if element['arrival_time'] < completion_time: # if uav arrive before completion time of activity -> no waiting time 
                waiting_time = 0
            
            if waiting_time >= truck.launch_time or waiting_time >= truck.delivery_time:
                #get first none retrival element 
                swappable = False
                for s_index in range(index, len(order_queue)):
                    s_element = order_queue[s_index]

                    if s_element['action'] == "retrieve":
                        continue

                    if s_element['action'] == "launch":
                        if s_element['element'] in retrieved_uavs:
                            #update complettion time - launch event will take place right after previous completion time
                            completion_time += truck.launch_time
                            s_element['completion_time'] = completion_time

                            order_queue.remove(s_element)
                            order_queue.insert(index, s_element)
                            swappable = True
                            break

                    if s_element['action'] == "delivery":
                        completion_time += truck.delivery_time
                        s_element['completion_time'] = completion_time

                        order_queue.remove(s_element)
                        order_queue.insert(index, s_element)
                        swappable = True
                        break
                        
                #if un-swappable --> waiting for uav to arrive and retrive it
                if not swappable:
                    completion_time += (waiting_time + truck.retrival_time)
                    element['completion_time'] = completion_time
                    retrieved_uavs.add(element['element'])

            else: #cannot insert element between uav's retrivals 
                retrieved_uavs.add(element['element'])

                #update completeion time 
                completion_time += (waiting_time + truck.retrival_time)
                element['completion_time'] = completion_time

        return order_queue, completion_time

    def local_search(self):
        pass

