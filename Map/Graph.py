from Map.Parameter import *
import math
import matplotlib.pyplot as plt


class Graph():
    def __init__(self):
        """
            Info about: 
            - Nodes
            - Edges
        """
        self.all_nodes = {} #all the nodes of the osm data
        self.node_label = 0

        self.nodes = set() #node of the road net work
        self.edges_dict = dict()
        pass

    def add_node(self, data):
        node = Node(-1, int(data['id']), data['lon'], data['lat'])
        self.all_nodes[int(data['id'])] = node
    
    def get_node(self, nId):
        return self.all_nodes[nId]

    def add_edge(self, data):
        nodes = data["nodes"]
        
        #for one way street
        if data["num_ways"] == 1:
            for i in range(len(nodes)):
                if i < len(nodes) - 1:
                    j = i + 1
                    srcId = int(nodes[i])
                    desId = int(nodes[j])
                    self.add_edge_logic(i, j, srcId, desId, data["num_ways"], data["type"])

        #for 2 way street
        elif data["num_ways"] == 2:
            for i in range(len(nodes)):
                if i < len(nodes) - 1:
                    j = i + 1
                    srcId = int(nodes[i])
                    desId = int(nodes[j])
                    self.add_edge_logic(i, j, srcId, desId, data["num_ways"], data["type"])

            #make arc in reverse order
            for i, e in reversed(list(enumerate(nodes))):
                if i > 0:
                    j = i - 1
                    srcId = int(nodes[i])
                    desId = int(nodes[j])
                    self.add_edge_logic(i, j, srcId, desId, data["num_ways"], data["type"])
    
    def add_edge_logic(self, i, j, srcId, desId, num_ways, rType):
        try:
            #get node from the dict
            src = self.get_node(srcId)
            
            des = self.get_node(desId)

         
            #create new edge
            edge = Edge(src, des)
            
            #add edge info
            edge.num_ways = num_ways
            edge.type = rType
            

            #append edge 
            self.edges_dict[(src, des)] = edge  
            
            #add node to the nodes list
            if i == 0:
                if src in self.nodes:
                    pass
                else:
                    #add to node list
                    self.nodes.add(src)
                    src.rep_id = self.node_label
                    self.node_label +=1
                
                if des in self.nodes:
                    pass
                else:
                    self.nodes.add(des)
                    des.rep_id = self.node_label
                    self.node_label +=1
                    
            else:
                if des in self.nodes:
                    pass
                else:
                    self.nodes.add(des)
                    des.rep_id = self.node_label
                    self.node_label +=1
                    

            #add desneigbor to be neigbor of src neigbor
            src.add_neigbor(des)
            
        except:
            print("not able to add edge")

    def get_edge(self, src, des):
        return self.edges_dict[(src, des)]

    def draw(self, dType = "num_ways"):
        for edge in self.edges_dict:
            arc = self.edges_dict[edge]

            x = [arc.src.lon, arc.des.lon]
            y = [arc.src.lat, arc.des.lat]

            if dType == "road_type":
                if arc.type == "motorway" or arc.type == "motorway_link" or arc.type == "trunk" or arc.type == "trunk_link":
                    plt.plot(x, y, 'purple', linewidth = 4)

                #primary and secondary
                if arc.type == "primary" or arc.type == "primary_link" or arc.type == "secondary" or arc.type == "secondary_link":
                    plt.plot(x, y, 'red', linewidth = 2)
            
                #tertiary and service
                elif arc.type == "tertiary" or arc.type == "tertiary_link" or arc.type == "service" or arc.type == "service_link":
                    plt.plot(x, y, 'orange', linewidth = 1)
                #other
                else:
                    plt.plot(x, y, 'lightgreen', linewidth = 1)
            elif dType == "num_ways":
                if arc.num_ways == 2:
                    plt.plot(x, y, 'lightgreen', linewidth = 1)
                elif arc.num_ways == 1:
                    plt.plot(x, y, 'red', linewidth = 1)

        for node in self.nodes:
            plt.plot(node.lon, node.lat, "ko", markersize= 12)
            plt.annotate(node.rep_id, (node.lon, node.lat), color= "w", ha= "center", va= "center", fontsize = 7)


class Node():
    def __init__(self, rep_id, nid, lon, lat):
        self.rep_id = rep_id
        self.id = nid
        self.lon = lon
        self.lat = lat

        self.neigbors = set()

    def distanceTo(self, n):
        
        #haversine phomular
        dLat = n.lat * math.pi / 180 - self.lat * math.pi / 180
        dLon = n.lon * math.pi / 180 - self.lon * math.pi / 180


        a = math.sin(dLat/2) * math.sin(dLat/2) + math.cos(self.lat * math.pi/180) * math.cos(n.lat * math.pi/180) * math.sin(dLon/2) * math.sin(dLon/2)
        c = 2* math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        d = R * c

        return d * 1000# km
    
    def add_neigbor(self, node):
        self.neigbors.add(node)

    
    def draw():
        pass

    def __str__(self):
        return f"Node {self.rep_id}"
    
    def __repr__(self):
        return f"Node {self.rep_id}"


class Edge():
    def __init__(self, src, des):
      
        self.src= src
        self.des = des
        self.length =  src.distanceTo(des)
       

        #congestion model
        self.speed_limit = SPEED_LIMIT
        self.num_ways = 2
        self.type = None
        self.direction = DIRECTION
        self.congestion_level = 0
