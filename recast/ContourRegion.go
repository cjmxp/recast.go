package recast

type ContourRegion struct {
	outline *Contour
	holes   []*ContourHole
	nholes  int
}
