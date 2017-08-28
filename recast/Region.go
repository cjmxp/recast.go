package recast

type Region struct {
	spanCount        int // Number of spans belonging to this region
	id               int // ID of the region
	areaType         int // Are type.
	remap            bool
	visited          bool
	overlap          bool
	connectsToBorder bool
	ymin             int
	ymax             int
	connections      []int
	floors           []int
}
