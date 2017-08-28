package recast

type ChunkyTriMesh struct {
	nodes           []*ChunkyTriMeshNode
	ntris           int
	maxTrisPerChunk int
}
type ChunkyTriMeshNode struct {
	bmin []float32
	bmax []float32
	tris []int
	i    int
}

type BoundsItem struct {
	bmin []float32
	bmax []float32
	i    int
}

func (this *ChunkyTriMesh) Init(verts []float32, tris []int, ntris int, trisPerChunk int) {
	//nchunks := (ntris + trisPerChunk - 1) / trisPerChunk
	this.nodes = []*ChunkyTriMeshNode{}
	// Build tree
	items := make([]*BoundsItem, ntris)
	for i := 0; i < ntris; i++ {
		t := i * 3
		it := &BoundsItem{}
		it.bmax = make([]float32, 2)
		it.bmin = make([]float32, 2)
		items[i] = it
		it.i = i
		// Calc triangle XZ bounds.
		it.bmax[0] = verts[tris[t]*3+0]
		it.bmin[0] = it.bmax[0]
		it.bmax[1] = verts[tris[t]*3+2]
		it.bmin[1] = it.bmax[1]
		for j := 1; j < 3; j++ {
			v := tris[t+j] * 3
			if verts[v] < it.bmin[0] {
				it.bmin[0] = verts[v]
			}
			if verts[v+2] < it.bmin[1] {
				it.bmin[1] = verts[v+2]
			}
			if verts[v] > it.bmax[0] {
				it.bmax[0] = verts[v]
			}
			if verts[v+2] > it.bmax[1] {
				it.bmax[1] = verts[v+2]
			}
		}
	}
	this.subdivide(items, 0, ntris, trisPerChunk, &this.nodes, tris)
	// Calc max tris per node.
	for i := 0; i < len(this.nodes); i++ {
		node := this.nodes[i]
		isLeaf := node.i >= 0
		if !isLeaf {
			continue
		}
		if len(node.tris)/3 > this.maxTrisPerChunk {
			this.maxTrisPerChunk = len(node.tris) / 3
		}
	}
}

func (this *ChunkyTriMesh) GetChunksOverlappingRect(bmin []float32, bmax []float32) []*ChunkyTriMeshNode {
	// Traverse tree
	ids := []*ChunkyTriMeshNode{}
	i := 0
	for i < len(this.nodes) {
		node := this.nodes[i]
		overlap := this.checkOverlapRect(bmin, bmax, node.bmin, node.bmax)
		isLeafNode := node.i >= 0
		if isLeafNode && overlap {
			ids = append(ids, node)
		}
		if overlap || isLeafNode {
			i++
		} else {
			i = -node.i
		}
	}
	return ids
}
func (this *ChunkyTriMesh) checkOverlapRect(amin []float32, amax []float32, bmin []float32, bmax []float32) bool {
	overlap := true
	if amin[0] > bmax[0] || amax[0] < bmin[0] {
		overlap = false
	}
	if amin[1] > bmax[1] || amax[1] < bmin[1] {
		overlap = false
	}
	return overlap
}
func (this *ChunkyTriMesh) subdivide(items []*BoundsItem, imin int, imax int, trisPerChunk int, nodes *[]*ChunkyTriMeshNode, inTris []int) {
	inum := imax - imin
	node := &ChunkyTriMeshNode{}
	node.bmin = make([]float32, 2)
	node.bmax = make([]float32, 2)
	node.tris = []int{}
	*nodes = append(*nodes, node)
	if inum <= trisPerChunk {
		// Leaf
		this.calcExtends(items, imin, imax, node.bmin, node.bmax)
		// Copy triangles.
		node.i = len(*nodes)
		node.tris = make([]int, inum*3)
		dst := 0
		for i := imin; i < imax; i++ {
			src := items[i].i * 3
			node.tris[dst] = inTris[src]
			dst++
			node.tris[dst] = inTris[src+1]
			dst++
			node.tris[dst] = inTris[src+2]
			dst++
		}
	} else {
		// Split
		this.calcExtends(items, imin, imax, node.bmin, node.bmax)
		axis := this.longestAxis(node.bmax[0]-node.bmin[0], node.bmax[1]-node.bmin[1])
		isort := &ISort{}
		if axis == 0 {
			isort.Data = items[imin:imax]
			isort.Call = func(a interface{}, b interface{}) bool {
				if a.(*BoundsItem).bmin[0] < b.(*BoundsItem).bmin[0] {
					return true
				}
				return false
			}
			isort.Stable()
			isort.Free()
			// Sort along x-axis
		} else if axis == 1 {
			isort.Data = items[imin:imax]
			isort.Call = func(a interface{}, b interface{}) bool {
				if a.(*BoundsItem).bmin[1] < b.(*BoundsItem).bmin[1] {
					return true
				}
				return false
			}
			isort.Stable()
			isort.Free()
			// Sort along y-axis
		}
		isplit := imin + inum/2
		// Left
		this.subdivide(items, imin, isplit, trisPerChunk, nodes, inTris)
		// Right
		this.subdivide(items, isplit, imax, trisPerChunk, nodes, inTris)
		// Negative index means escape.
		node.i = -len(*nodes)
	}
}
func (this *ChunkyTriMesh) calcExtends(items []*BoundsItem, imin int, imax int, bmin []float32, bmax []float32) {
	bmin[0] = items[imin].bmin[0]
	bmin[1] = items[imin].bmin[1]
	bmax[0] = items[imin].bmax[0]
	bmax[1] = items[imin].bmax[1]
	for i := imin + 1; i < imax; i++ {
		it := items[i]
		if it.bmin[0] < bmin[0] {
			bmin[0] = it.bmin[0]
		}
		if it.bmin[1] < bmin[1] {
			bmin[1] = it.bmin[1]
		}
		if it.bmax[0] > bmax[0] {
			bmax[0] = it.bmax[0]
		}
		if it.bmax[1] > bmax[1] {
			bmax[1] = it.bmax[1]
		}
	}
}
func (this *ChunkyTriMesh) longestAxis(x float32, y float32) int {
	if y > x {
		return 1
	}
	return 0
}
