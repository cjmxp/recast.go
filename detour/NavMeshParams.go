package detour

type NavMeshParams struct {
	/** The world space origin of the navigation mesh's tile space. [(x, y, z)] */
	orig []float32
	/** The width of each tile. (Along the x-axis.) */
	tileWidth float32
	/** The height of each tile. (Along the z-axis.) */
	tileHeight float32
	/** The maximum number of tiles the navigation mesh can contain. */
	maxTiles int
	/** The maximum number of polygons each tile can contain. */
	maxPolys int
}

func (this *NavMeshParams) Init(o []float32, w float32, h float32, mt int, mp int) {
	this.orig = append([]float32{}, o...)
	this.tileWidth = w
	this.tileHeight = h
	this.maxTiles = mt
	this.maxPolys = mp

}
