import osmium as osm
import pandas as pd


class MapReader(osm.SimpleHandler):
    def __init__(self, graph):
        osm.SimpleHandler.__init__(self)
        self.graph = graph
        self.nodes = []
       

    def node(self, n):
        #node format: n8440236617: location=105.850900/21.037167 tags={name=Bánh Ran Cõ Truyên Mât Đuòng,shop=bakery}
        #get road data
        
        data = {}
        data['id'] = n.id
       
        data['lon'] = n.location.lon
        data['lat'] = n.location.lat
        data['tags'] = n.tags
        self.graph.add_node(data)
      
            

    def way(self, w):
        #data type: w892981729: nodes=[8299505505,8299505504,8299505503,8299505502,829...] tags={highway=residential,motorcar=no,motorcycle=yes} 
        if w.tags.get("highway"):
            #get node id 
            data = {}
            nodes = []
            for n in w.nodes:
                nodes.append(int(n.ref))

            data['nodes'] = nodes
            
            #get one way or 2 way
            if w.tags.get("oneway"):
                data["num_ways"] = 1
            else:
                data["num_ways"] = 2

            #get road type
            data["type"] = w.tags.get("highway") 
            #footway, residential, secondary, primary_link, primary, tertiary, service, tertiary_link, steps, path, pedestrian
            
            self.graph.add_edge(data)

    def relation(self, r):
        pass

