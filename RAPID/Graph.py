from . import utils

import numpy as np
import matplotlib.pyplot as plt
#from scipy.ndimage import binary_dilation, generate_binary_structure
from scipy.ndimage import distance_transform_edt
from skimage.morphology import skeletonize, medial_axis

from collections import defaultdict

#from PIL import Image

#TODO : faire une structure plus complexe pour les noeuds maintenant permettant de leur attribuer une zone, des artefacts, etc.

class Node():
    def __init__(self, coordinates, agent, neighbors, explored, zone, type, creation_time):
        """
        Docstring for __init__
        
        :param self: Description
        :param id: Description
        :param coordinates: Description
        :param agent: Description
        :param neighbors: Description
        :param explored: Description
        :param zone: Description
        :param type: Description
        :param creation_time: Description
        """
        self.coordinates = coordinates
        self.agents = [agent]
        self.neighbors = neighbors
        self.explored = explored
        self.zone = zone
        self.type = type
        self.creation_time = creation_time

    def add_neighbor(self, id, distance):
        self.neighbors.update({id:distance})
    
    def remove_neighbor(self, id):
        del self.neighbors[id]



class Graph():
    def __init__(self, occupancy_grid, agent_id, nodes_distance_treshold=5):
        #TODO graph creation
        self.agent_id = agent_id
        self.dist_treshold = nodes_distance_treshold
        self.graph = None
        self.nodes = {}
        self.graph_generation(occupancy_grid)

        
    def graph_generation(self, grid):
        def euclidean_distance_transform(grid):
            # grid : tableau 2D où 0 = obstacle, 1 = espace libre

            # Créer une copie de la grille pour ne pas modifier l'originale
            grid_with_borders = grid.copy()

            # Marquer les bords comme obstacles (0)
            grid_with_borders[0, :] = 1  # Bord supérieur
            grid_with_borders[-1, :] = 1  # Bord inférieur
            grid_with_borders[:, 0] = 1  # Bord gauche
            grid_with_borders[:, -1] = 1  # Bord droit


            dist_grid =  distance_transform_edt(grid_with_borders == 0)

            return dist_grid #dist_grid
        
        def build_voronoi_graph(skeleton, dist_tf_grid):
            # making a graph from skeletton
            graph = defaultdict(Node)
            width, height = skeleton.shape
            directions = [(-1, -1), (-1, 0), (-1, 1),
                        (0, -1),          (0, 1),
                        (1, -1),  (1, 0), (1, 1)]

            # foreach pixels of the skeletton
            for x in range(1, width - 1):
                for y in range(1, height - 1):
                    if skeleton[x, y]:
                        #on cree le node
                        id=(x,y)
                        self.nodes.update({id :Node(
                            coordinates= (x,y),
                            agent = self.agent_id,
                            neighbors={},
                            explored=False,
                            zone = None, #TODO
                            type=None, #TODO
                            creation_time=None #TODO
                            ) })
                        neighbors = []
                        for dx, dy in directions:
                            nx, ny = x + dx, y + dy
                            if 0 <= nx < width and 0 <= ny < height and skeleton[nx, ny]:
                                neighbors.append((nx, ny))

                        #self.add_node(id=id, neighbors=neighbors)
                        for neighbor in neighbors:
                            self.nodes[id].add_neighbor(neighbor, utils.euclidian_distance((x,y), neighbor) )


            # Étape 3 : Idetify node of degree 2 or 3
            degree_2_nodes = []
            degree_3_nodes = []
            critical_points = []

            for node in self.nodes:
                if len(self.nodes[node].neighbors) ==1:
                    critical_points.append(node)
                if len(self.nodes[node].neighbors) == 2:
                    degree_2_nodes.append(node)
                elif len(self.nodes[node].neighbors) >= 3:
                    degree_3_nodes.append(node)
                    critical_points.append(node)
            
            for node in degree_2_nodes:
                # check if there is a neighbor of degree 3
                for neighbor in self.nodes[node].neighbors:
                    if neighbor in degree_3_nodes:
                        critical_points.append(node)
                        break
            
            
            # adding local minimas of dist as critical points
            for node in self.nodes:
                distances_neighbors = []
                for neighbor in self.nodes[node].neighbors:
                    distances_neighbors.append(dist_tf_grid[neighbor])

                if dist_tf_grid[node] < np.min(distances_neighbors):
                    critical_points.append(node)


            # remove non critical nodes
            non_critical_nodes = [node for node in self.nodes if node not in critical_points]

            for node in non_critical_nodes:

                neighbors = self.nodes[node].neighbors
                if len(neighbors) == 2:
                    # handling the connections of the node we remove
                    neighbor1, neighbor2 = neighbors
                    if neighbor2 not in self.nodes[neighbor1].neighbors:
                        self.nodes[neighbor1].add_neighbor(id=neighbor2, distance=utils.euclidian_distance(neighbor1, neighbor2) )
                    if neighbor1 not in self.nodes[neighbor2].neighbors:
                        self.nodes[neighbor2].add_neighbor(id=neighbor1, distance=utils.euclidian_distance(neighbor1, neighbor2) )
                # then delete the node
                self.remove_node(id=node)
                #del graph[node]

            return self.nodes, critical_points
        
        distance_map = euclidean_distance_transform(grid=grid)
        skeleton = skeletonize(distance_map)
        build_voronoi_graph(skeleton=skeleton, dist_tf_grid=distance_map)

        self.clean_graph()
    
    def remove_node(self, id):
        for neighbor in self.nodes[id].neighbors:
            del self.nodes[neighbor].neighbors[id]
        del self.nodes[id]

    def add_node(self, id, neighbors=None):
        """
         Adding a Node to the graph.
        
        :param id: (x,y) tuple id ; coordinate of a node.
        :param neighbors: List of (x,y) id tuples, will generate the associated neighbors in the node neighbors attribute.
        """
        new_neighbors= {}
        for n in neighbors:
            dist = utils.euclidian_distance(id, n)
            new_neighbors.update({n:dist}) #maybe we should verify if those nodes are effectively in the graph
            #on ajoute le noeud en nouveau voisin aussi
            self.nodes[n].add_neighbor(id=id, distance=dist)

        self.nodes.update({
            id: Node(
                coordinates=id,
                agent=self.agent_id,
                neighbors= new_neighbors,
                explored=False, #TODO
                zone=None, #TODO
                type = None, #TODO
                creation_time=None #TODO
            )
        })
        pass

    def clean_graph(self):
        """
        clean_graph is used to clean the graph. The nodes that are to close to each other will be fused depending on the graph "distance_treshold" attribut.
        """
        changed = True

        while changed:
            nodes = list(self.nodes.keys()) #copy pour changer le dico tranquille
            changed = False
            for node in nodes:
                x, y = node
                if node in self.nodes:
                    neighbors = list(self.nodes[node].neighbors.keys())

                    for neighbor in neighbors:
                        nx, ny = neighbor
                        if utils.euclidian_distance((x, y), (nx, ny)) <= self.dist_treshold and node in self.nodes:
                            # Calcul du point moyen
                            moyen = ((x + nx) / 2, (y + ny) / 2)

                            # Récupération des autres voisins
                            other_neighbors = []
                            for other in self.nodes[node].neighbors:
                                if other != neighbor:
                                    other_neighbors.append(other)
                            for other in self.nodes[neighbor].neighbors:
                                if other != node and other not in other_neighbors:
                                    other_neighbors.append(other)

                            # Suppression des anciens nœuds et ajout du nouveau
                            for other in other_neighbors:
                                if node in self.nodes[other].neighbors:
                                    self.nodes[other].add_neighbor(id=moyen, distance=utils.euclidian_distance(other, moyen) )
                                if neighbor in self.nodes[other].neighbors:
                                    self.nodes[other].add_neighbor(id=moyen, distance=utils.euclidian_distance(other, moyen) )
                                
                            # Ajout du nœud moyen au graph
                            self.add_node(moyen, other_neighbors)
                            

                            # Suppression des anciens nœuds
                            self.remove_node(node)
                            self.remove_node(neighbor)

                            changed = True

        return None
    
    def plot_voronoi_graph(self, img=None):
        from matplotlib import pyplot as plt
        # Afficher le squelette en arrière-plan
        if img:
            plt.imshow(img, interpolation='nearest', cmap='binary', alpha=0.5)

        # Tracer les arêtes du graphe
        for node in self.nodes:
            x, y = node
            for neighbor in self.nodes[node].neighbors:
                nx, ny = neighbor
                plt.plot([x, nx], [y, ny], 'b-', linewidth=0.5)  # Tracer les arêtes en bleu

        # Tracer les nœuds du graphe
        for node in self.nodes:
            x, y = node
            plt.plot(x, y, 'ro', markersize=3)  # Tracer les nœuds en rouge

        # Ajouter une légende et un titre
        plt.show()