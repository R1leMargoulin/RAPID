from . import utils

import numpy as np
import heapq
import matplotlib.pyplot as plt
#from scipy.ndimage import binary_dilation, generate_binary_structure
from scipy.ndimage import distance_transform_edt
from skimage.morphology import skeletonize, medial_axis
from skimage.measure import find_contours, approximate_polygon

from shapely.geometry import Point, MultiPoint
from shapely.geometry.polygon import Polygon

from collections import defaultdict

#from PIL import Image

#TODO : ajouter des artefacts aux nodes??? pas necessairement besoin car on a les coordonnees des artefacts, on peut les relier via la zone.

class Node():
    def __init__(self, coordinates, agent:list, creation_time, zone=None, neighbors={}, type= None, explored = False):
        """
        Docstring for __init__
        
        :param self: Description
        :param id: Description
        :param coordinates: Description
        :param agent: Description
        :param neighbors: Description {id : dist}
        :param explored: Description
        :param zone: Description
        :param type: Description
        :param creation_time: Description
        """
        self.coordinates = coordinates
        self.agents = agent
        self.neighbors = neighbors
        self.explored = explored
        self.zone = zone
        self.type = type
        self.creation_time = creation_time

    def add_neighbor(self, id, distance):
        self.neighbors.update({id:distance})
    
    def remove_neighbor(self, id):
        del self.neighbors[id]
    
    def get_zone_points(self):
        polygon = Polygon(self.zone)

        #generate all possible points in bounds
        xmin, ymin, xmax, ymax = polygon.bounds
        x = np.arange(np.floor(xmin), np.ceil(xmax) + 1)
        y = np.arange(np.floor(ymin), np.ceil(ymax) + 1)
        points = MultiPoint(np.transpose([np.tile(x, len(y)), np.repeat(y, len(x))])) 

        result = points.intersection(polygon) #keeps only the points that intersects with the polygon

        coordinates = [(point.x, point.y) for point in result.geoms] #to get an (x,y) tuple list.
        return coordinates



class Graph():
    def __init__(self, occupancy_grid, agent_id, nodes_distance_treshold=5, traversable_types=[0]):
        self.agent_id = agent_id
        self.dist_treshold = nodes_distance_treshold
        #graph itself
        self.graph = None
        self.nodes = {}
        self.graph_generation(occupancy_grid, traversable_types=traversable_types) #pour les unknown, peut etre mettre ca en parametrable... a voir

        #polygon zone allocation
        nodelist = list(self.nodes.keys())
        allocaton_map = self.allocate_cells_to_nodes(nodelist, occupancy_grid, traversable_types=traversable_types)
        polygones = self.extraire_polygones(allocaton_map, nodelist)
        for p in polygones:
            self.nodes[p].zone = polygones[p][0]
    
    def graph_generation(self, grid, traversable_types=[0]):
        def euclidean_distance_transform(grid):
            # grid : tableau 2D où 0 = obstacle, 1 = espace libre

            # Créer une copie de la grille pour ne pas modifier l'originale
            grid_with_borders = grid.copy()

            # Marquer les bords comme obstacles (0)
            #-2 = absolutely untraversable, while -1 = unknown.
            grid_with_borders[0, :] = -2  # Bord sup
            grid_with_borders[-1, :] = -2  # Bord inf
            grid_with_borders[:, 0] = -2  # Bord gauche
            grid_with_borders[:, -1] = -2  # Bord droit

            traversable_with_unknown = traversable_types
            
            #dist_grid =  distance_transform_edt(grid_with_borders == 0) #for binary grid

            dist_grid =  distance_transform_edt(np.isin(grid_with_borders, traversable_with_unknown))

            return dist_grid #dist_grid
        
        def build_voronoi_graph(skeleton, dist_tf_grid):
            # making a graph from skeletton
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
                            agent = [self.agent_id],
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
                if len(self.nodes[node].neighbors) == 0:
                    continue
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
        
        self.nodes = {}
        distance_map = euclidean_distance_transform(grid=grid)
        skeleton = skeletonize(distance_map)
        build_voronoi_graph(skeleton=skeleton, dist_tf_grid=distance_map)

        self.clean_graph()
    
    def remove_node(self, id):
        for neighbor in self.nodes[id].neighbors:
            if neighbor in self.nodes:
                if id in self.nodes[neighbor].neighbors:
                    del self.nodes[neighbor].neighbors[id]
        del self.nodes[id]

    def add_node(self, id, neighbors=None, observers=None):
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

        if observers == None:
            observer_agents = [self.agent_id]
        else:
            observer_agents = observers

        self.nodes.update({
            id: Node(
                coordinates=id,
                agent=observer_agents,
                neighbors= new_neighbors,
                explored=False, #TODO
                zone=None, #TODO
                type = None, #TODO
                creation_time=None #TODO
            )
        })
        pass

    def update_graph(self, occupancy_grid, agent_id, traversable_types=[0]):
        nodes_backup = self.nodes
        #getting unseen nodes to keep those in the graph after regeneration
        unseen_nodes = {}
        for nb in nodes_backup:
            if agent_id not in nodes_backup[nb].agents:
                unseen_nodes.update({nb:nodes_backup[nb]})
        #graph regeneration
        self.graph_generation(occupancy_grid)
        self.allocate_polygons(occupancy_grid, traversable_types)

        
        
        #2 juste les ajouter et les relier au noeud le plus proche du coisin original. 
        #Si y'a un tres gros nombre de noeuds, jsp si ca ralentira beaucoup ou pas le truc. a tester
        for unseen in unseen_nodes:
            seen_neighbors = []
            neighbors = unseen_nodes[unseen].neighbors
            for n in list(neighbors.keys()):
                if n not in list(unseen_nodes.keys()): #si le noeud a ete vu par le robot au final, alors il doit etre dans le graph, mais il peut avoir bouge legerement d'ou ce trickshot...
                    seen_neighbors.append(n)
            for seen in seen_neighbors:
                closest_node = None
                min_dist = np.inf
                for node in self.nodes:
                    dist = utils.euclidian_distance(seen, self.nodes[node].coordinates)
                    if dist < min_dist:
                        min_dist = dist
                        closest_node = self.nodes[node].coordinates
                del unseen_nodes[unseen].neighbors[seen] #on vire le voisin deja vu au cas ou
                unseen_nodes[unseen].neighbors.update({closest_node:utils.euclidian_distance(closest_node, unseen)}) #on ajoute le noeud identifie en tant que voisin
            #au final on ajoute simplement le noeud au graph, et normalement ca marche (spoiler oui)
            self.nodes.update({unseen : unseen_nodes[unseen]})

    def clean_graph(self):
        """
        clean_graph is used to clean the graph. The nodes that are to close to each other will be fused depending on the graph "distance_treshold" attribut.
        """
        changed = True

        while changed:
            nodes = list(self.nodes.keys()) #copy pour changer le dico tranquille
            changed = False
            for node in nodes:

                if node in self.nodes:
                    observer_agents = self.nodes[node].agents
                else:
                    observer_agents = []

                x, y = node
                if node in self.nodes:
                    neighbors = list(self.nodes[node].neighbors.keys())

                    deg2neighbors = []

                    for neighbor in neighbors: #TODO TODO TODO IL FAUT MERGE CERTAINS NOEUDS MEME SI PAS VOISINS DIRECT, CERTAINS NE SONT PAS MERGED...
                        if neighbor in self.nodes:
                            deg2neighbors = deg2neighbors + list(self.nodes[neighbor].neighbors.keys())
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
                                    
                                observer_agents = list(np.unique( observer_agents + self.nodes[neighbor].agents)) #merge agents that have seen those two merged nodes

                                # Ajout du nœud moyen au graph
                                self.add_node(moyen, other_neighbors, observers=observer_agents)
                                

                                # Suppression des anciens nœuds
                                self.remove_node(node)
                                self.remove_node(neighbor)

                                changed = True
                    #             break
                    # if changed:
                    #     break
                    
                       
                    # for deg2 in deg2neighbors:
                    #     if deg2 == node or deg2 not in self.nodes:
                    #         continue
                    #     nx, ny = deg2

                    #     if utils.euclidian_distance((x, y), (nx, ny)) <= self.dist_treshold*2 and node in self.nodes:
                    #         # Calcul du point moyen
                    #         moyen = ((x + nx) / 2, (y + ny) / 2)

                    #         # Récupération des autres voisins
                    #         other_neighbors = []
                    #         for other in self.nodes[node].neighbors:
                    #             if other != deg2 and other in self.nodes:
                    #                 other_neighbors.append(other)
                    #         for other in self.nodes[deg2].neighbors:
                    #             if other != node and other not in other_neighbors and other in self.nodes:
                    #                 other_neighbors.append(other)

                    #         # Suppression des anciens nœuds et ajout du nouveau
                    #         for other in other_neighbors:
                    #             # if other not in self.nodes:
                    #             #     continue
                    #             if node in self.nodes[other].neighbors:
                    #                 self.nodes[other].add_neighbor(id=moyen, distance=utils.euclidian_distance(other, moyen) )
                    #             if deg2 in self.nodes[other].neighbors:
                    #                 self.nodes[other].add_neighbor(id=moyen, distance=utils.euclidian_distance(other, moyen) )
                                
                    #         observer_agents = list(np.unique( observer_agents + self.nodes[deg2].agents)) #merge agents that have seen those two merged nodes

                    #         # Ajout du nœud moyen au graph
                    #         self.add_node(moyen, other_neighbors, observers=observer_agents)
                            

                    #         # Suppression des anciens nœuds
                    #         self.remove_node(node)
                    #         self.remove_node(deg2)

                    #         changed = True
                    #         break
                    # if changed:
                    #     break
                    

        return None
    
    def merge_graph(self, new_graph): 
        for node in new_graph.nodes:
            if node not in self.nodes:
                nodelist = list(self.nodes.keys())
                for localnode in nodelist:
                    if utils.euclidian_distance(node, localnode) < self.dist_treshold:
                        #merge the two nodes directly
                        # Calcul du point moyen
                        moyen = ((node[0] + localnode[0]) / 2, (node[1] + localnode[1]) / 2)

                        # Récupération des autres voisins
                        other_neighbors = []
                        for other in self.nodes[localnode].neighbors:
                            if other != localnode and other in self.nodes:
                                other_neighbors.append(other)
                        for other in new_graph.nodes[node].neighbors:
                            if other != localnode and other not in other_neighbors and other in self.nodes:
                                other_neighbors.append(other)

                        # Suppression des anciens nœuds et ajout du nouveau
                        for other in other_neighbors:
                            # if other not in self.nodes:
                            #     continue
                            if localnode in self.nodes[other].neighbors:
                                self.nodes[other].add_neighbor(id=moyen, distance=utils.euclidian_distance(other, moyen) )
                            if node in self.nodes[other].neighbors:
                                self.nodes[other].add_neighbor(id=moyen, distance=utils.euclidian_distance(other, moyen) )
                            
                        observer_agents = list(np.unique( self.nodes[localnode].agents + new_graph.nodes[node].agents)) #merge agents that have seen those two merged nodes

                        # Ajout du nœud moyen au graph
                        self.add_node(moyen, other_neighbors, observers=observer_agents)
                        

                        # Suppression des anciens nœuds
                        self.remove_node(localnode)
                        #self.remove_node(deg2)

                #self.nodes.update({node:new_graph.nodes[node]})
            else:
                self.nodes[node].agents = list(np.unique(new_graph.nodes[node].agents + self.nodes[node].agents)) #merging of the agents that has seen that node
                
        
        self.clean_graph()
        pass
    
    def plot_voronoi_graph(self, img=None, display_zones=False):
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
            if display_zones:
                try:
                    plt.plot(*zip(*self.nodes[node].zone) )#'k'
                except Exception as e:
                    print(e)
                    continue

        # Ajouter une légende et un titre
        plt.show()

    def allocate_polygons(self, occupancy_grid, traversable_types):
        nodelist=[]
        external_nodes = []
        for node in self.nodes:
            print(self.nodes[node].zone)
            if self.nodes[node].zone == None:
                nodelist.append(node)
            else:
                external_nodes.append(node)
        print(nodelist)
        print(external_nodes)
        allocaton_map = self.allocate_cells_to_nodes(nodelist, occupancy_grid, traversable_types=traversable_types)

        for en in external_nodes:
            zonepoints = self.nodes[en].get_zone_points
            for p in zonepoints:
                allocaton_map[p] = -1

        polygones = self.extraire_polygones(allocaton_map, nodelist)
        for p in polygones:
            self.nodes[p].zone = polygones[p][0]

    def allocate_cells_to_nodes(self, node_list, matrice_occupation, traversable_types=[0]):
        """allocate each free cell to the closest node using a WPA for distance and obstacle avoidement"""
            
        # Initialisation
        allocation_matrix = np.full(matrice_occupation.shape, -1, dtype=int)
        distance_matrix = np.full(matrice_occupation.shape, np.inf)
        open_set = []

        # Ajouter les nœuds à la file de priorité (distance 0)
        for node_idx, (ny, nx) in enumerate(node_list):
            heapq.heappush(open_set, (0, int(ny), int(nx), node_idx))

        # Directions possibles (4-connexité)
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        while open_set:
            current_dist, y, x, node_idx = heapq.heappop(open_set)

            # Si la distance actuelle est supérieure à celle enregistrée, on passe
            if current_dist > distance_matrix[y, x]:
                continue

            # Explorer les voisins
            for dy, dx in directions:
                ny, nx = y + dy, x + dx

                # Vérifier les limites et les murs
                if (0 <= ny < matrice_occupation.shape[0] and
                    0 <= nx < matrice_occupation.shape[1] and
                    matrice_occupation[ny, nx] in traversable_types + [-1] and  # Pas un mur
                    distance_matrix[ny, nx] > current_dist + 1):

                    distance_matrix[ny, nx] = current_dist + 1
                    allocation_matrix[ny, nx] = node_idx
                    heapq.heappush(open_set, (distance_matrix[ny, nx], ny, nx, node_idx))
        #print(allocation_matrix)

        return allocation_matrix
    
    def extraire_polygones(self, matrice, node_list):

        matrice_etendue = np.pad(matrice, pad_width=1, mode='constant', constant_values=-1)

        polygones = {}
        for value in range(len(node_list)):
            # if value == -1:  # Ignorer la bordure ajoutée et les valeurs a -1
            #     continue

            contours = find_contours(matrice_etendue == value)

            
            polygones[node_list[value]] = []
            for contour in contours:
                # (y, x) -> (x, y) 
                contour_corrige = contour[:, :] - 1  # -1 cause of padding
                contour_corrige = np.trunc(contour_corrige).astype(int)

                # Fermer le polygone en reliant le premier et dernier point si nécessaire
                if not np.array_equal(contour_corrige[0], contour_corrige[-1]):
                    contour_corrige = np.vstack([contour_corrige, contour_corrige[0]])

                # Simplifier le polygone avec Douglas-Peucker
                contour_simplifie = approximate_polygon(contour_corrige, tolerance=0.8) #Douglas-peucker

                polygones[node_list[value]].append(contour_simplifie.tolist())
                #polygones[value].append(contour_corrige.tolist())

        return polygones
    
    def identify_polygon_to_point(self, point:tuple):
        spoint = Point(point[0], point[1])
        identified = None
        for node in self.nodes:
            polygon = Polygon(self.nodes[node].zone) #zone of a point is a polygon
            if polygon.contains(spoint):
                identified = node
                break
        return identified

    def check_explored_nodes(self, occupancy_grid):
        for node in self.nodes:
            pointlist = self.nodes[node].get_zone_points()
            for p in pointlist:
                #TODO : check if one of those points is unknown (aka == -1) !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                pass