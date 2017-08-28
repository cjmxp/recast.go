package detour

type Node struct {
	index int
	/** Position of the node. */
	pos []float32
	/** Cost from previous node to current node. */
	cost float32
	/** Cost up to the node. */
	total float32
	/** Index to parent node. */
	pidx int
	/** extra state information. A polyRef can have multiple nodes with different extra info. see DT_MAX_STATES_PER_NODE */
	state int
	/** Node flags. A combination of dtNodeFlags. */
	flags int
	/** Polygon ref the node corresponds to. */
	id int64
}

func (this *Node) Init(index int) {
	this.index = index
	this.pos = make([]float32, 3)
}
func (this *Node) Less(other interface{}) bool {
	return this.total < other.(*Node).total
}
