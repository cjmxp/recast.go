package detour

type MeshTile struct {
	index int
	/** Counter describing modifications to the tile. */
	salt int
	/** The tile data. */
	data *MeshData
	/** The tile links. */
	links []*Link
	/** Index to the next free link. */
	linksFreeList int
	/** Tile flags. (See: #dtTileFlags) */
	flags int
	/** The next free tile, or the next tile in the spatial grid. */
	next *MeshTile
}

func (this *MeshTile) Init(index int) {
	this.index = index
	this.linksFreeList = DT_NULL_LINK
}
