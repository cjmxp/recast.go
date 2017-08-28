package recast

import "math"

func ErodeWalkableArea(radius int, chf *CompactHeightfield) {
	w := chf.width
	h := chf.height
	dist := make([]int, chf.spanCount)
	for i := 0; i < chf.spanCount; i++ {
		dist[i] = 255
	}
	// Mark boundary cells.
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			c := &chf.cells[x+y*w]
			for i, ni := c.index, c.index+c.count; i < ni; i++ {
				if chf.areas[i] == RC_NULL_AREA {
					dist[i] = 0
				} else {
					s := &chf.spans[i]
					nc := 0
					for dir := 0; dir < 4; dir++ {
						if GetCon(s, dir) != RC_NOT_CONNECTED {
							nx := x + GetDirOffsetX(dir)
							ny := y + GetDirOffsetY(dir)
							nidx := chf.cells[nx+ny*w].index + GetCon(s, dir)
							if chf.areas[nidx] != RC_NULL_AREA {
								nc++
							}
						}
					}
					// At least one missing neighbour.
					if nc != 4 {
						dist[i] = 0
					}
				}
			}
		}
	}
	nd := 0
	// Pass 1
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			c := &chf.cells[x+y*w]
			for i, ni := c.index, c.index+c.count; i < ni; i++ {
				s := &chf.spans[i]
				if GetCon(s, 0) != RC_NOT_CONNECTED {
					// (-1,0)
					ax := x + GetDirOffsetX(0)
					ay := y + GetDirOffsetY(0)
					ai := chf.cells[ax+ay*w].index + GetCon(s, 0)
					as := &chf.spans[ai]
					nd = int(math.Min(float64(dist[ai]+2), 255))
					if nd < dist[i] {
						dist[i] = nd
					}
					// (-1,-1)
					if GetCon(as, 3) != RC_NOT_CONNECTED {
						aax := ax + GetDirOffsetX(3)
						aay := ay + GetDirOffsetY(3)
						aai := chf.cells[aax+aay*w].index + GetCon(as, 3)
						nd = int(math.Min(float64(dist[aai]+3), 255))
						if nd < dist[i] {
							dist[i] = nd
						}
					}
				}
				if GetCon(s, 3) != RC_NOT_CONNECTED {
					// (0,-1)
					ax := x + GetDirOffsetX(3)
					ay := y + GetDirOffsetY(3)
					ai := chf.cells[ax+ay*w].index + GetCon(s, 3)
					as := &chf.spans[ai]
					nd = int(math.Min(float64(dist[ai]+2), 255))
					if nd < dist[i] {
						dist[i] = nd
					}
					// (1,-1)
					if GetCon(as, 2) != RC_NOT_CONNECTED {
						aax := ax + GetDirOffsetX(2)
						aay := ay + GetDirOffsetY(2)
						aai := chf.cells[aax+aay*w].index + GetCon(as, 2)
						nd = int(math.Min(float64(dist[aai]+3), 255))
						if nd < dist[i] {
							dist[i] = nd
						}
					}
				}
			}
		}
	}
	// Pass 2
	for y := h - 1; y >= 0; y-- {
		for x := w - 1; x >= 0; x-- {
			c := &chf.cells[x+y*w]
			for i, ni := c.index, c.index+c.count; i < ni; i++ {
				s := &chf.spans[i]
				if GetCon(s, 2) != RC_NOT_CONNECTED {
					// (1,0)
					ax := x + GetDirOffsetX(2)
					ay := y + GetDirOffsetY(2)
					ai := chf.cells[ax+ay*w].index + GetCon(s, 2)
					as := &chf.spans[ai]
					nd = int(math.Min(float64(dist[ai]+2), 255))
					if nd < dist[i] {
						dist[i] = nd
					}
					// (1,1)
					if GetCon(as, 1) != RC_NOT_CONNECTED {
						aax := ax + GetDirOffsetX(1)
						aay := ay + GetDirOffsetY(1)
						aai := chf.cells[aax+aay*w].index + GetCon(as, 1)
						nd = int(math.Min(float64(dist[aai]+3), 255))
						if nd < dist[i] {
							dist[i] = nd
						}
					}
				}
				if GetCon(s, 1) != RC_NOT_CONNECTED {
					// (0,1)
					ax := x + GetDirOffsetX(1)
					ay := y + GetDirOffsetY(1)
					ai := chf.cells[ax+ay*w].index + GetCon(s, 1)
					as := &chf.spans[ai]
					nd = int(math.Min(float64(dist[ai]+2), 255))
					if nd < dist[i] {
						dist[i] = nd
					}
					// (-1,1)
					if GetCon(as, 0) != RC_NOT_CONNECTED {
						aax := ax + GetDirOffsetX(0)
						aay := ay + GetDirOffsetY(0)
						aai := chf.cells[aax+aay*w].index + GetCon(as, 0)
						nd = int(math.Min(float64(dist[aai]+3), 255))
						if nd < dist[i] {
							dist[i] = nd
						}
					}
				}
			}
		}
	}
	thr := radius * 2
	for i := 0; i < chf.spanCount; i++ {
		if dist[i] < thr {
			chf.areas[i] = RC_NULL_AREA
		}
	}
}

func markConvexPolyArea(verts []float32, hmin float32, hmax float32, areaId int, chf *CompactHeightfield) {
	bmin := make([]float32, 3)
	bmax := make([]float32, 3)
	copy3(bmin, 0, verts, 0)
	copy3(bmax, 0, verts, 0)
	for i := 1; i < len(verts); i++ {
		min(bmin, verts, i*3)
		max(bmax, verts, i*3)
	}
	bmin[1] = hmin
	bmax[1] = hmax
	minx := int((bmin[0] - chf.bmin[0]) / chf.cs)
	miny := int((bmin[1] - chf.bmin[1]) / chf.ch)
	minz := int((bmin[2] - chf.bmin[2]) / chf.cs)
	maxx := int((bmax[0] - chf.bmin[0]) / chf.cs)
	maxy := int((bmax[1] - chf.bmin[1]) / chf.ch)
	maxz := int((bmax[2] - chf.bmin[2]) / chf.cs)
	if maxx < 0 {
		return
	}
	if minx >= chf.width {
		return
	}
	if maxz < 0 {
		return
	}
	if minz >= chf.height {
		return
	}
	if minx < 0 {
		minx = 0
	}
	if maxx >= chf.width {
		maxx = chf.width - 1
	}
	if minz < 0 {
		minz = 0
	}
	if maxz >= chf.height {
		maxz = chf.height - 1
	}
	// TODO: Optimize.
	for z := minz; z <= maxz; z++ {
		for x := minx; x <= maxx; x++ {
			c := &chf.cells[x+z*chf.width]
			for i, ni := c.index, c.index+c.count; i < ni; i++ {
				s := &chf.spans[i]
				if chf.areas[i] == RC_NULL_AREA {
					continue
				}
				if s.y >= miny && s.y <= maxy {
					p := make([]float32, 3)
					p[0] = chf.bmin[0] + (float32(x)+0.5)*chf.cs
					p[1] = 0
					p[2] = chf.bmin[2] + (float32(z)+0.5)*chf.cs
					if pointInPoly(verts, p) {
						chf.areas[i] = areaId
					}
				}
			}
		}
	}
}
func pointInPoly(verts []float32, p []float32) bool {
	c := false
	i, j := 0, 0
	for i, j = 0, len(verts)-1; i < len(verts); j, i = i, i+1 {
		vi := i * 3
		vj := j * 3
		if ((verts[vi+2] > p[2]) != (verts[vj+2] > p[2])) && (p[0] < (verts[vj]-verts[vi])*(p[2]-verts[vi+2])/(verts[vj+2]-verts[vi+2])+verts[vi]) {
			c = !c
		}
	}
	return c
}
