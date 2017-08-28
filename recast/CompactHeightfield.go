package recast

type CompactHeightfield struct {
	/** The width of the heightfield. (Along the x-axis in cell units.) */
	width int
	/** The height of the heightfield. (Along the z-axis in cell units.) */
	height int
	/** The number of spans in the heightfield. */
	spanCount int
	/** The walkable height used during the build of the field.  (See: RecastConfig::walkableHeight) */
	walkableHeight int
	/** The walkable climb used during the build of the field. (See: RecastConfig::walkableClimb) */
	walkableClimb int
	/** The AABB border size used during the build of the field. (See: RecastConfig::borderSize) */
	borderSize int
	/** The maximum distance value of any span within the field. */
	maxDistance int
	/** The maximum region id of any span within the field. */
	maxRegions int
	/** The minimum bounds in world space. [(x, y, z)] */
	bmin []float32
	/** The maximum bounds in world space. [(x, y, z)] */
	bmax []float32
	/** The size of each cell. (On the xz-plane.) */
	cs float32
	/** The height of each cell. (The minimum increment along the y-axis.) */
	ch float32
	/** Array of cells. [Size: #width*#height] */
	cells []CompactCell
	/** Array of spans. [Size: #spanCount] */
	spans []CompactSpan
	/** Array containing border distance data. [Size: #spanCount] */
	dist []int
	/** Array containing area id data. [Size: #spanCount] */
	areas []int
}

func (this *CompactHeightfield) Get_maxDistance() int {
	return this.maxDistance
}
func (this *CompactHeightfield) Get_maxRegions() int {
	return this.maxRegions
}
