# dbscan.py

from math import sqrt

class DBScan:
    def __init__(self, eps=1.0, min_pts=3):
        """
        eps: distanza massima per considerare due punti vicini
        min_pts: numero minimo di punti nel vicinato per essere un core
        """
        self.eps = eps
        self.min_pts = min_pts
        self.clusters = [] 

    def fit(self, cells):
        """
        cells: lista di tuple (i, j) rappresentanti celle 2D
        Ritorna: lista di celle rappresentative (una per cluster)
        """
        self.clusters = []  
        if not cells:
            return []

        visited = set()
        cluster_id = 0
        representatives = []

        for p in cells:
            if p in visited:
                continue

            visited.add(p)
            neighbors = self._region_query(p, cells)

            if len(neighbors) >= self.min_pts:
                cluster_id += 1
                representatives.append(p)
                new_cluster = [p]
                self._expand_cluster(p, neighbors, visited, cells, new_cluster)
                self.clusters.append(new_cluster)

        return representatives

    def _expand_cluster(self, p, neighbors, visited, cells, cluster_points):
        """
        Espande un cluster a partire dal punto p e dalla sua lista di vicini.
        cluster_points: lista che accumula i punti di questo cluster
        """
        i = 0
        while i < len(neighbors):
            n_point = neighbors[i]
            if n_point not in visited:
                visited.add(n_point)
                cluster_points.append(n_point)
                n_neighbors = self._region_query(n_point, cells)
                if len(n_neighbors) >= self.min_pts:
                    neighbors.extend(n_neighbors)
            elif n_point not in cluster_points:
                cluster_points.append(n_point)
            i += 1

    def _region_query(self, p, cells):
        """
        Trova tutti i punti in 'cells' entro distanza <= eps da p.
        """
        res = []
        for q in cells:
            if self._distance(p, q) <= self.eps:
                res.append(q)
        return res

    def _distance(self, p1, p2):
        """
        Distanza euclidea tra due celle (i, j).
        """
        return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

"""

# ESEMPIO D'USO
if __name__ == "__main__":
    cells = [
        (0, 0), (0, 1), (1, 0), (5, 5),
        (5, 6), (6, 5), (10, 10)
    ]

    dbscan = DBScan(eps=1.5, min_pts=3)
    reps = dbscan.fit(cells)
    print("Celle rappresentative:", reps)
    print("Cluster trovati:", dbscan.clusters)

"""
