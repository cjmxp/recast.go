package recast

type RecastBuilderResult struct {
	pmesh *PolyMesh
	dmesh *PolyMeshDetail
}

func (this *RecastBuilderResult) Init(pmesh *PolyMesh, dmesh *PolyMeshDetail) {
	this.pmesh = pmesh
	this.dmesh = dmesh
	for i := 0; i < this.pmesh.Get_npolys(); i++ {
		this.pmesh.flags[i] = 1
	}
}
func (this *RecastBuilderResult) GetMesh() *PolyMesh {
	return this.pmesh
}

func (this *RecastBuilderResult) GetMeshDetail() *PolyMeshDetail {
	return this.dmesh
}
