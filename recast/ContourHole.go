package recast

type ContourHole struct {
	leftmost int
	minx     int
	minz     int
	contour  *Contour
}
