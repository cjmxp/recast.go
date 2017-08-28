package detour

type BVNode struct {
	/** Minimum bounds of the node's AABB. [(x, y, z)] */
	bmin []int
	/** Maximum bounds of the node's AABB. [(x, y, z)] */
	bmax []int
	/** The node's index. (Negative for escape sequence.) */
	i int
}

func (this *BVNode) Init() {
	this.bmin = make([]int, 3)
	this.bmax = make([]int, 3)
}
