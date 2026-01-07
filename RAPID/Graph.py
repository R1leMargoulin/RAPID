import utils

import numpy as np
import matplotlib.pyplot as plt
#from scipy.ndimage import binary_dilation, generate_binary_structure
from scipy.ndimage import distance_transform_edt
from skimage.morphology import skeletonize, medial_axis

from collections import defaultdict

#from PIL import Image

#TODO : faire une structure plus complexe pour les noeuds maintenant permettant de leur attribuer une zone, des artefacts, etc.

class Graph():
    def __init__(self, occupancy_grid, nodes_distance_treshold=5):
        #TODO graph creation
        self.dist_treshold = nodes_distance_treshold
        self.graph = None
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
            graph = defaultdict(list)
            width, height = skeleton.shape
            directions = [(-1, -1), (-1, 0), (-1, 1),
                        (0, -1),          (0, 1),
                        (1, -1),  (1, 0), (1, 1)]

            # foreach pixels of the skeletton
            for x in range(1, width - 1):
                for y in range(1, height - 1):
                    if skeleton[x, y]:
                        neighbors = []
                        for dx, dy in directions:
                            nx, ny = x + dx, y + dy
                            if 0 <= nx < width and 0 <= ny < height and skeleton[nx, ny]:
                                neighbors.append((nx, ny))

                        for neighbor in neighbors:
                            graph[(x, y)].append(neighbor)

            # Étape 3 : Idetify node of degree 2 or 3
            degree_2_nodes = []
            degree_3_nodes = []
            critical_points = []

            for node in graph:
                if len(graph[node]) ==1:
                    critical_points.append(node)
                if len(graph[node]) == 2:
                    degree_2_nodes.append(node)
                elif len(graph[node]) >= 3:
                    degree_3_nodes.append(node)
                    critical_points.append(node)

            # Critical points identifications
            
            for node in degree_2_nodes:
                # check if there is a neighbor of degree 3
                for neighbor in graph[node]:
                    if neighbor in degree_3_nodes:
                        critical_points.append(node)
                        break
            
            # adding local minimas of dist as critical points
            for node in graph:
                distances_neighbors = []
                for neighbor in graph[node]:
                    distances_neighbors.append(dist_tf_grid[neighbor])

                if dist_tf_grid[node] < np.min(distances_neighbors):
                    critical_points.append(node)


            # remove non critical nodes
            non_critical_nodes = [node for node in graph if node not in critical_points]

            for node in non_critical_nodes:

                neighbors = graph[node]
                if len(neighbors) == 2:
                    # handling the connections of the node we remove
                    neighbor1, neighbor2 = neighbors
                    if neighbor2 not in graph[neighbor1]:
                        graph[neighbor1].append(neighbor2)
                        graph[neighbor1].remove(node)
                    if neighbor1 not in graph[neighbor2]:
                        graph[neighbor2].append(neighbor1)
                        graph[neighbor2].remove(node)
                elif len(neighbors) ==1:
                    graph[neighbors[0]].remove(node)
                    graph[node].remove(neighbors[0])
                # then delete the node
                del graph[node]

            return graph, critical_points
        
        distance_map = euclidean_distance_transform(grid=grid)
        skeleton = skeletonize(distance_map)
        self.graph, _ =  build_voronoi_graph(skeleton=skeleton, dist_tf_grid=distance_map)

        self.clean_graph()

    def clean_graph(self):
        changed = True

        while changed:
            nodes = list(self.graph.keys()) #copy pour changer le dico tranquille
            changed = False
            for node in nodes:
                x, y = node
                if node in self.graph:
                    neighbors = list(self.graph[node])

                    for neighbor in neighbors:
                        nx, ny = neighbor
                        if utils.euclidian_distance((x, y), (nx, ny)) <= self.dist_threshold:
                            # Calcul du point moyen
                            moyen = ((x + nx) / 2, (y + ny) / 2)

                            # Récupération des autres voisins
                            other_neighbors = []
                            for other in self.graph[node]:
                                if other != neighbor:
                                    other_neighbors.append(other)
                            for other in self.graph[neighbor]:
                                if other != node and other not in other_neighbors:
                                    other_neighbors.append(other)

                            # Suppression des anciens nœuds et ajout du nouveau
                            for other in other_neighbors:
                                if node in self.graph[other]:
                                    self.graph[other].remove(node)
                                    self.graph[other].append(moyen)
                                if neighbor in self.graph[other]:
                                    self.graph[other].remove(neighbor)
                                    self.graph[other].append(moyen)
                                self.graph[moyen].append(other)

                            # # Ajout du nœud moyen au graph
                            # graph[moyen] = []
                            # for other in other_neighbors:
                            #    graph[moyen].append(other)
                            

                            # Suppression des anciens nœuds
                            del self.graph[node]
                            del self.graph[neighbor]
                            changed = True

        return None
    
    def plot_voronoi_graph(self, img=None):
        from matplotlib import pyplot as plt
        # Afficher le squelette en arrière-plan
        if img:
            plt.imshow(img, interpolation='nearest', cmap='binary', alpha=0.5)

        # Tracer les arêtes du graphe
        for node in self.graph:
            x, y = node
            for neighbor in self.graph[node]:
                nx, ny = neighbor
                plt.plot([x, nx], [y, ny], 'b-', linewidth=0.5)  # Tracer les arêtes en bleu

        # Tracer les nœuds du graphe
        for node in self.graph:
            x, y = node
            plt.plot(x, y, 'ro', markersize=3)  # Tracer les nœuds en rouge

        # Ajouter une légende et un titre
        plt.show()