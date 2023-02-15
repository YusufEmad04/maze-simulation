class TileNode:
    def __init__(self, value: tuple, visited=True):
        self.quadrants = value
        self.   visited = visited
        self.neighbors = set()

    def __str__(self):
        return f"TileNode({self.quadrants}, {self.visited}), {id(self)}"

    def add_neighbor(self, neighbor):
        self.neighbors.add(neighbor)


class TilesGraph:
    def __init__(self):
        self.tiles = dict()

    def __str__(self):
        cat = ""
        for tile in self.tiles:
            cat += f"{self.tiles[tile]}\n"
        return cat

    def remove_all_occurrences(self, quadrant):
        for tile in self.tiles:
            self.tiles[tile].neighbors.discard(self.tiles[quadrant])
        del self.tiles[quadrant]


    def add_node(self, tile: TileNode, neighbour_tile: TileNode):

        tile_is_new = tile.quadrants not in self.tiles
        neighbour_tile_is_new = neighbour_tile.quadrants not in self.tiles

        if tile_is_new:
            self.tiles[tile.quadrants] = tile
            tile_ref = self.tiles[tile.quadrants]
        else:
            tile_ref = self.tiles[tile.quadrants]

        if neighbour_tile_is_new:
            self.tiles[neighbour_tile.quadrants] = neighbour_tile
            neighbour_tile_ref = self.tiles[neighbour_tile.quadrants]
        else:
            neighbour_tile_ref = self.tiles[neighbour_tile.quadrants]

        tile_ref.add_neighbor(neighbour_tile_ref)
        tile_ref.visited = True
        neighbour_tile_ref.add_neighbor(tile_ref)


        # if tile.quadrants not in self.tiles:
        #     self.tiles[tile.quadrants] = tile
        # else:
        #     self.tiles[tile.quadrants].visited = True
        #
        # if neighbour_tile.quadrants not in self.tiles:
        #     self.tiles[neighbour_tile.quadrants] = neighbour_tile
        #
        # neighbour_tile.add_neighbor(tile)
        # tile.add_neighbor(neighbour_tile)

        # ----------------- OLD -----------------

        # self.tiles[tile.quadrants] = tile
        # if not neighbour_tile.visited and neighbour_tile.quadrants not in self.tiles:
        #     self.tiles[neighbour_tile.quadrants] = neighbour_tile
        #     neighbour_tile.add_neighbor(tile)
        # tile.add_neighbor(neighbour_tile)


