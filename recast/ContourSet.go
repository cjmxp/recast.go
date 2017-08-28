package recast

type ContourSet struct {
	/** A list of the contours in the set. */
	conts []*Contour
	/** The minimum bounds in world space. [(x, y, z)] */
	bmin []float32
	/** The maximum bounds in world space. [(x, y, z)] */
	bmax []float32
	/** The size of each cell. (On the xz-plane.) */
	cs float32
	/** The height of each cell. (The minimum increment along the y-axis.) */
	ch float32
	/** The width of the set. (Along the x-axis in cell units.) */
	width int
	/** The height of the set. (Along the z-axis in cell units.) */
	height int
	/** The AABB border size used to generate the source data from which the contours were derived. */
	borderSize int
	/** The max edge error that this contour set was simplified with. */
	maxError float32
}

func (this *ContourSet) Get_counts() int {
	return len(this.conts)
}
