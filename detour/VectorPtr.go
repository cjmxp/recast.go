package detour

type VectorPtr struct {
	array []float32
	index int
}

func (this *VectorPtr) Init(array []float32, index int) {
	this.array = array
	this.index = index
}
func (this *VectorPtr) Get(offset int) float32 {
	return this.array[this.index+offset]
}
