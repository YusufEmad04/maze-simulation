class TileNode:
    def __init__(self, value: tuple, visited=True):
        self.quadrants = value
        self.visited = visited
        self.neighbors = []

    def __str__(self):
        return f"TileNode({self.quadrants}, {self.visited})"

    def add_neighbor(self, neighbor):
        self.neighbors.append(neighbor)


class TilesGraph:
    def __init__(self):
        self.tiles = dict()

    def __str__(self):
        cat = ""
        for tile in self.tiles:
            cat += f"{self.tiles[tile]}\n"
        return cat

    def add_node(self, tile: TileNode, neighbour_tile: TileNode):
        self.tiles[tile.quadrants] = tile
        if not neighbour_tile.visited and neighbour_tile.quadrants not in self.tiles:
            self.tiles[neighbour_tile.quadrants] = neighbour_tile
            neighbour_tile.add_neighbor(tile)
        tile.add_neighbor(neighbour_tile)


