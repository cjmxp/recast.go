package recast

type InputGeom struct {
	vertices []float32
	faces    []int
	bmin     []float32
	bmax     []float32
}

func (this *InputGeom) Init(vertexPositions []float32, meshFaces []int) {
	this.bmin = make([]float32, 3)
	this.bmax = make([]float32, 3)
	this.vertices = []float32{}
	this.faces = []int{}
	this.vertices = append(this.vertices, vertexPositions...)
	this.faces = append(this.faces, meshFaces...)
	copy3(this.bmin, 0, this.vertices, 0)
	copy3(this.bmax, 0, this.vertices, 0)
	for i := 1; i < len(this.vertices)/3; i++ {
		min(this.bmin, this.vertices, i*3)
		max(this.bmax, this.vertices, i*3)
	}
}
func (this *InputGeom) GetChunkyMesh() *ChunkyTriMesh {
	ct := &ChunkyTriMesh{}
	ct.Init(this.vertices, this.faces, len(this.faces)/3, 256)
	return ct
}
func (this *InputGeom) GetMeshBoundsMin() []float32 {
	return append([]float32{}, this.bmin...)
}

func (this *InputGeom) GetMeshBoundsMax() []float32 {
	return append([]float32{}, this.bmax...)
}

func (this *InputGeom) GetVerts() []float32 {
	return this.vertices
}
func (this *InputGeom) GetTris() []int {
	return this.faces
}
func (this *InputGeom) Load() {

}
func (this *InputGeom) GetConvexVolumes() []*ConvexVolume {
	return []*ConvexVolume{}
}
