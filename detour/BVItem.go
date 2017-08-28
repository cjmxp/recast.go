package detour

type BVItem struct {
	bmin []int
	bmax []int
	i    int
}

func (this *BVItem) Init() {
	this.bmin = make([]int, 3)
	this.bmax = make([]int, 3)
}
