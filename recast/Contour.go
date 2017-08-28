package recast

type Contour struct {
	/** Simplified contour vertex and connection data. [Size: 4 * #nverts] */
	verts []int
	/** The number of vertices in the simplified contour. */
	nverts int
	/** Raw contour vertex and connection data. [Size: 4 * #nrverts] */
	rverts []int
	/** The number of vertices in the raw contour.  */
	nrverts int
	/** The region id of the contour. */
	area int
	/** The area id of the contour. */
	reg int
}
