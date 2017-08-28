package recast

type RecastBuilderConfig struct {
	cfg *RecastConfig
	/** The width of the field along the x-axis. [Limit: >= 0] [Units: vx] **/
	width int

	/** The height of the field along the z-axis. [Limit: >= 0] [Units: vx] **/
	height int

	/** The minimum bounds of the field's AABB. [(x, y, z)] [Units: wu] **/
	bmin []float32

	/** The maximum bounds of the field's AABB. [(x, y, z)] [Units: wu] **/
	bmax []float32

	/** The size of the non-navigable border around the heightfield. [Limit: >=0] [Units: vx] **/
	borderSize int

	/** Set to true for tiled build **/
	tiled bool
	tx    int
	ty    int
}

func (this *RecastBuilderConfig) Init(cfg *RecastConfig, bmin []float32, bmax []float32, tx int, ty int, tiled bool) {
	this.cfg = cfg
	this.tiled = tiled
	this.bmin = []float32{}
	this.bmax = []float32{}
	this.bmin = append(this.bmin, bmin...)
	this.bmax = append(this.bmax, bmax...)
	this.tx = tx
	this.ty = ty
	if tiled {
		ts := float32(cfg.TileSize) * cfg.Cs
		this.bmin[0] += float32(tx) * ts
		this.bmin[2] += float32(ty) * ts
		this.bmax[0] = this.bmin[0] + ts
		this.bmax[2] = this.bmin[2] + ts

		// Expand the heighfield bounding box by border size to find the extents of geometry we need to build this
		// tile.
		//
		// This is done in order to make sure that the navmesh tiles connect correctly at the borders,
		// and the obstacles close to the border work correctly with the dilation process.
		// No polygons (or contours) will be created on the border area.
		//
		// IMPORTANT!
		//
		// :''''''''':
		// : +-----+ :
		// : | | :
		// : | |<--- tile to build
		// : | | :
		// : +-----+ :<-- geometry needed
		// :.........:
		//
		// You should use this bounding box to query your input geometry.
		//
		// For example if you build a navmesh for terrain, and want the navmesh tiles to match the terrain tile size
		// you will need to pass in data from neighbour terrain tiles too! In a simple case, just pass in all the 8
		// neighbours,
		// or use the bounding box below to only pass in a sliver of each of the 8 neighbours.

		this.borderSize = cfg.WalkableRadius + 3 // Reserve enough padding.
		this.bmin[0] -= float32(this.borderSize) * cfg.Cs
		this.bmin[2] -= float32(this.borderSize) * cfg.Cs
		this.bmax[0] += float32(this.borderSize) * cfg.Cs
		this.bmax[2] += float32(this.borderSize) * cfg.Cs
		this.width = cfg.TileSize + this.borderSize*2
		this.height = cfg.TileSize + this.borderSize*2
	} else {
		w, h := calcGridSize(this.bmin, this.bmax, cfg.Cs)
		this.width = w
		this.height = h
		this.borderSize = 0
	}
}

func (this *RecastBuilderConfig) GetWidth() int {
	return this.width
}

func (this *RecastBuilderConfig) GetHeight() int {
	return this.height
}

func (this *RecastBuilderConfig) GetBmin() []float32 {
	return this.bmin
}
func (this *RecastBuilderConfig) GetBmax() []float32 {
	return this.bmax
}
