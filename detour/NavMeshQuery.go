package detour

import "math"

type NavMeshQuery struct {
	nav          *NavMesh
	nodePool     *NodePool
	tinyNodePool *NodePool
	openList     *NodeQueue
	query        *QueryData /// < Sliced query state.
}

func (this *NavMeshQuery) Init(nav *NavMesh) {
	this.nav = nav
	this.nodePool = &NodePool{}
	this.nodePool.Init()
	this.tinyNodePool = &NodePool{}
	this.tinyNodePool.Init()
	this.openList = &NodeQueue{}
	this.openList.Init()
}
func (this *NavMeshQuery) FindPath(startRef int64, endRef int64, startPos []float32, endPos []float32, filter *QueryFilter) (int, []int64) {
	if startRef == 0 || endRef == 0 {
		panic("Start or end ref = 0")
	}
	// Validate input
	if !this.nav.isValidPolyRef(startRef) || !this.nav.isValidPolyRef(endRef) {
		panic("Invalid start or end ref")
	}
	if startRef == endRef {
		return SUCCSESS, []int64{startRef}
	}
	this.nodePool.clear()
	this.openList.clear()
	startNode := this.nodePool.getNode(startRef)
	vCopy(startNode.pos, startPos, 0)
	startNode.pidx = 0
	startNode.cost = 0
	startNode.total = vDist(startPos, endPos, 0) * H_SCALE
	startNode.id = startRef
	startNode.flags = DT_NODE_OPEN
	this.openList.push(startNode)
	lastBestNode := startNode
	lastBestNodeCost := startNode.total
	status := SUCCSESS
	for !this.openList.isEmpty() {
		// Remove node from open list and put it in closed list.
		bestNode := this.openList.pop()
		bestNode.flags &= ^DT_NODE_OPEN
		bestNode.flags |= DT_NODE_CLOSED
		// Reached the goal, stop searching.
		if bestNode.id == endRef {
			lastBestNode = bestNode
			break
		}
		// Get current poly and tile.
		// The API input has been cheked already, skip checking internal data.
		bestRef := bestNode.id
		bestTile, bestPoly := this.nav.getTileAndPolyByRefUnsafe(bestRef)
		// Get parent poly and tile.
		parentRef := int64(0)
		var parentTile *MeshTile = nil
		var parentPoly *Poly = nil
		if bestNode.pidx != 0 {
			parentRef = this.nodePool.getNodeAtIdx(bestNode.pidx).id
		}
		if parentRef != 0 {
			parentTile, parentPoly = this.nav.getTileAndPolyByRefUnsafe(parentRef)
		}
		for i := bestPoly.firstLink; i != DT_NULL_LINK; i = bestTile.links[i].next {
			neighbourRef := bestTile.links[i].ref
			// Skip invalid ids and do not expand back to where we came from.
			if neighbourRef == 0 || neighbourRef == parentRef {
				continue
			}
			// Get neighbour poly and tile.
			// The API input has been cheked already, skip checking internal data.
			neighbourTile, neighbourPoly := this.nav.getTileAndPolyByRefUnsafe(neighbourRef)
			if !filter.passFilter(neighbourRef, neighbourTile, neighbourPoly) {
				continue
			}
			// deal explicitly with crossing tile boundaries
			crossSide := 0
			if bestTile.links[i].side != 0xff {
				crossSide = bestTile.links[i].side >> 1
			}
			// get the node
			neighbourNode := this.nodePool.getNode2(neighbourRef, crossSide)
			// If the node is visited the first time, calculate node position.
			if neighbourNode.flags == 0 {
				neighbourNode.pos = this.getEdgeMidPoint(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly, neighbourTile)
			}
			// Calculate cost and heuristic.
			cost, heuristic := float32(0), float32(0)
			// Special case for last node.
			if neighbourRef == endRef {
				// Cost
				curCost := filter.getCost(bestNode.pos, neighbourNode.pos, parentRef, parentTile, parentPoly, bestRef, bestTile, bestPoly, neighbourRef, neighbourTile, neighbourPoly)
				endCost := filter.getCost(neighbourNode.pos, endPos, bestRef, bestTile, bestPoly, neighbourRef, neighbourTile, neighbourPoly, 0, nil, nil)
				cost = bestNode.cost + curCost + endCost
				heuristic = 0
			} else {
				// Cost
				curCost := filter.getCost(bestNode.pos, neighbourNode.pos, parentRef, parentTile, parentPoly, bestRef, bestTile, bestPoly, neighbourRef, neighbourTile, neighbourPoly)
				cost = bestNode.cost + curCost
				heuristic = vDist(neighbourNode.pos, endPos, 0) * H_SCALE
			}
			total := cost + heuristic
			// The node is already in open list and the new result is worse, skip.
			if (neighbourNode.flags&DT_NODE_OPEN) != 0 && total >= neighbourNode.total {
				continue
			}
			// The node is already visited and process, and the new result is worse, skip.
			if (neighbourNode.flags&DT_NODE_CLOSED) != 0 && total >= neighbourNode.total {
				continue
			}
			// Add or update the node.
			neighbourNode.pidx = this.nodePool.getNodeIdx(bestNode)
			neighbourNode.id = neighbourRef
			neighbourNode.flags = neighbourNode.flags & ^DT_NODE_CLOSED
			neighbourNode.cost = cost
			neighbourNode.total = total
			if (neighbourNode.flags & DT_NODE_OPEN) != 0 {
				// Already in open, update node location.
				this.openList.modify(neighbourNode)
			} else {
				// Put the node in open list.
				neighbourNode.flags |= DT_NODE_OPEN
				this.openList.push(neighbourNode)
			}
			// Update nearest node to target so far.
			if heuristic < lastBestNodeCost {
				lastBestNodeCost = heuristic
				lastBestNode = neighbourNode
			}
		}
	}
	path := this.getPathToNode(lastBestNode)
	if lastBestNode.id != endRef {
		status = PARTIAL_RESULT
	}
	return status, path
}
func (this *NavMeshQuery) FindStraightPath(startPos []float32, endPos []float32, path []int64, maxStraightPath int, options int) []*StraightPathItem {
	if len(path) == 0 {
		panic("Empty path")
	}
	// TODO: Should this be callers responsibility?
	closestStartPos := this.closestPointOnPolyBoundary(path[len(path)-1], startPos)
	closestEndPos := this.closestPointOnPolyBoundary(path[0], endPos)
	straightPath := []*StraightPathItem{}
	// Add start point.
	stat := this.appendVertex(closestStartPos, DT_STRAIGHTPATH_START, path[len(path)-1], &straightPath, maxStraightPath)
	if stat != IN_PROGRESS {
		return straightPath
	}
	if len(path) > 1 {
		portalApex := append([]float32{}, closestStartPos...)
		portalLeft := append([]float32{}, portalApex...)
		portalRight := append([]float32{}, portalApex...)
		apexIndex := 0
		leftIndex := 0
		rightIndex := 0
		leftPolyType := 0
		rightPolyType := 0
		leftPolyRef := path[len(path)-1]
		rightPolyRef := leftPolyRef
		for i := len(path) - 1; i >= 0; i-- {
			left := make([]float32, 3)
			right := make([]float32, 3)
			toType := 0
			if i-1 >= 0 {
				if portalPoints := this.getPortalPoints2(path[i], path[i-1]); portalPoints != nil {
					left = portalPoints.left
					right = portalPoints.right
					toType = portalPoints.toType
				} else {
					closestEndPos = this.closestPointOnPolyBoundary(path[i], endPos)
					if (options & (DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS)) != 0 {
						stat = this.appendPortals(apexIndex, i, closestEndPos, path, &straightPath, options, maxStraightPath)
						if stat != IN_PROGRESS {
							return straightPath
						}
					}
					this.appendVertex(closestEndPos, 0, path[i], &straightPath, maxStraightPath)
					return straightPath
				}
				// If starting really close the portal, advance.
				if i == 0 {
					_, second := distancePtSegSqr2D3(portalApex, left, right)
					if second < sqr(0.001) {
						continue
					}
				}
			} else {
				// End of the path.
				vCopy(left, closestEndPos, 0)
				vCopy(right, closestEndPos, 0)
				toType = DT_POLYTYPE_GROUND
			}
			// Right vertex.
			if triArea2D(portalApex, portalRight, right) <= 0.0 {
				if vEqual(portalApex, portalRight) || triArea2D(portalApex, portalLeft, right) > 0.0 {
					vCopy(portalRight, right, 0)
					rightPolyRef = 0
					if i-1 >= 0 {
						rightPolyRef = path[i-1]
					}
					rightPolyType = toType
					rightIndex = i
				} else {
					// Append portals along the current straight path segment.
					if (options & (DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS)) != 0 {
						stat = this.appendPortals(apexIndex, leftIndex, portalLeft, path, &straightPath, options, maxStraightPath)
						if stat != IN_PROGRESS {
							return straightPath
						}
					}
					vCopy(portalApex, portalLeft, 0)
					apexIndex = leftIndex
					flags := 0
					if leftPolyRef == 0 {
						flags = DT_STRAIGHTPATH_END
					} else if leftPolyType == DT_POLYTYPE_OFFMESH_CONNECTION {
						flags = DT_STRAIGHTPATH_OFFMESH_CONNECTION
					}
					ref := leftPolyRef
					// Append or update vertex
					stat = this.appendVertex(portalApex, flags, ref, &straightPath, maxStraightPath)
					if stat != IN_PROGRESS {
						return straightPath
					}
					vCopy(portalLeft, portalApex, 0)
					vCopy(portalRight, portalApex, 0)
					leftIndex = apexIndex
					rightIndex = apexIndex
					// Restart
					i = apexIndex
					continue
				}
			}
			// Left vertex.
			if triArea2D(portalApex, portalLeft, left) >= 0.0 {
				if vEqual(portalApex, portalLeft) || triArea2D(portalApex, portalRight, left) < 0.0 {
					vCopy(portalLeft, left, 0)
					leftPolyRef = 0
					if i-1 >= 0 {
						leftPolyRef = path[i-1]
					}
					leftPolyType = toType
					leftIndex = i
				} else {
					// Append portals along the current straight path segment.
					if (options & (DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS)) != 0 {
						stat = this.appendPortals(apexIndex, rightIndex, portalRight, path, &straightPath, options, maxStraightPath)
						if stat != IN_PROGRESS {
							return straightPath
						}
					}
					vCopy(portalApex, portalRight, 0)
					apexIndex = rightIndex

					flags := 0
					if rightPolyRef == 0 {
						flags = DT_STRAIGHTPATH_END
					} else if rightPolyType == DT_POLYTYPE_OFFMESH_CONNECTION {
						flags = DT_STRAIGHTPATH_OFFMESH_CONNECTION
					}

					ref := rightPolyRef
					// Append or update vertex
					stat = this.appendVertex(portalApex, flags, ref, &straightPath, maxStraightPath)
					if stat != IN_PROGRESS {
						return straightPath
					}
					vCopy(portalLeft, portalApex, 0)
					vCopy(portalRight, portalApex, 0)
					leftIndex = apexIndex
					rightIndex = apexIndex
					// Restart
					i = apexIndex
					continue
				}
			}
		}
		// Append portals along the current straight path segment.
		if (options & (DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS)) != 0 {
			stat = this.appendPortals(apexIndex, 0, closestEndPos, path, &straightPath, options, maxStraightPath)
			if stat != IN_PROGRESS {
				return straightPath
			}
		}
	}
	this.appendVertex(closestEndPos, DT_STRAIGHTPATH_END, 0, &straightPath, maxStraightPath)
	return straightPath
}
func (this *NavMeshQuery) appendPortals(startIdx int, endIdx int, endPos []float32, path []int64, straightPath *[]*StraightPathItem, maxStraightPath int, options int) int {
	startPos := (*straightPath)[len(*straightPath)-1].pos
	// Append or update last vertex
	stat := -1
	for i := endIdx - 1; i >= startIdx; i-- {
		// Calculate portal
		from := path[i]
		fromTile, fromPoly := this.nav.getTileAndPolyByRef(from)
		to := path[i-1]
		toTile, toPoly := this.nav.getTileAndPolyByRef(to)
		portals := this.getPortalPoints(from, fromPoly, fromTile, to, toPoly, toTile, 0, 0)
		left := portals.left
		right := portals.right
		if (options & DT_STRAIGHTPATH_AREA_CROSSINGS) != 0 {
			// Skip intersection if only area crossings are requested.
			if fromPoly.getArea() == toPoly.getArea() {
				continue
			}
		}
		// Append intersection
		first, _, third := intersectSegSeg2D(startPos, endPos, left, right)
		if first {
			pt := vLerp3(left, right, third)
			stat = this.appendVertex(pt, 0, path[i-1], straightPath, maxStraightPath)
			if stat != IN_PROGRESS {
				return stat
			}
		}
	}
	return IN_PROGRESS
}
func (this *NavMeshQuery) appendVertex(pos []float32, flags int, ref int64, straightPath *[]*StraightPathItem, maxStraightPath int) int {
	if len(*straightPath) > 0 && vEqual((*straightPath)[len(*straightPath)-1].pos, pos) {
		(*straightPath)[len(*straightPath)-1].flags = flags
		(*straightPath)[len(*straightPath)-1].ref = ref
	} else {
		if len(*straightPath) < maxStraightPath {
			sp := &StraightPathItem{}
			sp.Init(pos, flags, ref)
			*straightPath = append(*straightPath, sp)
		}
		// If reached end of path or there is no space to append more vertices, return.
		if flags == DT_STRAIGHTPATH_END || len(*straightPath) >= maxStraightPath {
			return SUCCSESS
		}
	}
	return IN_PROGRESS
}

func (this *NavMeshQuery) closestPointOnPolyBoundary(ref int64, pos []float32) []float32 {
	tile, poly := this.nav.getTileAndPolyByRef(ref)
	// Collect vertices.
	verts := make([]float32, this.nav.getMaxVertsPerPoly()*3)
	edged := make([]float32, this.nav.getMaxVertsPerPoly())
	edget := make([]float32, this.nav.getMaxVertsPerPoly())
	nv := poly.vertCount
	for i := 0; i < nv; i++ {
		l := poly.verts[i] * 3
		arr := tile.data.verts[l : l+3]
		copy(verts[i*3:i*3+3], arr)
	}
	closest := make([]float32, 3)
	if distancePtPolyEdgesSqr(pos, verts, nv, edged, edget) {
		vCopy(closest, pos, 0)
	} else {
		// Point is outside the polygon, dtClamp to nearest edge.
		dmin := edged[0]
		imin := 0
		for i := 1; i < nv; i++ {
			if edged[i] < dmin {
				dmin = edged[i]
				imin = i
			}
		}
		va := imin * 3
		vb := ((imin + 1) % nv) * 3
		closest = vLerp(verts, va, vb, edget[imin])
	}
	return closest
}

func (this *NavMeshQuery) getPathToNode(endNode *Node) []int64 {
	buff := []int64{}
	curNode := endNode
	for curNode != nil {
		buff = append(buff, curNode.id)
		curNode = this.nodePool.getNodeAtIdx(curNode.pidx)
	}
	path := []int64{}
	for i := len(buff) - 1; i >= 0; i-- {
		path = append(path, buff[i])
	}
	return buff
}
func (this *NavMeshQuery) getEdgeMidPoint(from int64, fromPoly *Poly, fromTile *MeshTile, to int64, toPoly *Poly, toTile *MeshTile) []float32 {
	ppoints := this.getPortalPoints(from, fromPoly, fromTile, to, toPoly, toTile, 0, 0)
	if ppoints == nil {
		panic("Null link")
	}
	left := ppoints.left
	right := ppoints.right
	mid := make([]float32, 3)
	mid[0] = (left[0] + right[0]) * 0.5
	mid[1] = (left[1] + right[1]) * 0.5
	mid[2] = (left[2] + right[2]) * 0.5
	return mid
}
func (this *NavMeshQuery) getPortalPoints2(from, to int64) *PortalResult {
	fromTile, fromPoly := this.nav.getTileAndPolyByRef(from)
	fromType := fromPoly.getType()
	toTile, toPoly := this.nav.getTileAndPolyByRef(to)
	toType := toPoly.getType()
	return this.getPortalPoints(from, fromPoly, fromTile, to, toPoly, toTile, fromType, toType)
}
func (this *NavMeshQuery) getPortalPoints(from int64, fromPoly *Poly, fromTile *MeshTile, to int64, toPoly *Poly, toTile *MeshTile, fromType int, toType int) *PortalResult {
	left := make([]float32, 3)
	right := make([]float32, 3)
	// Find the link that points to the 'to' polygon.
	var link *Link = nil
	for i := fromPoly.firstLink; i != DT_NULL_LINK; i = fromTile.links[i].next {
		if fromTile.links[i].ref == to {
			link = fromTile.links[i]
			break
		}
	}
	if link == nil {
		return nil
	}
	// Handle off-mesh connections.
	if fromPoly.getType() == DT_POLYTYPE_OFFMESH_CONNECTION {
		// Find link that points to first vertex.
		for i := fromPoly.firstLink; i != DT_NULL_LINK; i = fromTile.links[i].next {
			if fromTile.links[i].ref == to {
				v := fromTile.links[i].edge
				arr := fromTile.data.verts[fromPoly.verts[v]*3 : fromPoly.verts[v]*3+3]
				vCopy(left, arr, 0)
				vCopy(right, arr, 0)
				return &PortalResult{left, right, fromType, toType}
			}
		}
		panic("Invalid offmesh from connection")
	}
	if toPoly.getType() == DT_POLYTYPE_OFFMESH_CONNECTION {
		for i := toPoly.firstLink; i != DT_NULL_LINK; i = toTile.links[i].next {
			if toTile.links[i].ref == from {
				v := toTile.links[i].edge
				arr := toTile.data.verts[toPoly.verts[v]*3 : toPoly.verts[v]*3+3]
				vCopy(left, arr, 0)
				vCopy(right, arr, 0)
				return &PortalResult{left, right, fromType, toType}
			}
		}
		panic("Invalid offmesh to connection")
	}
	// Find portal vertices.
	v0 := fromPoly.verts[link.edge]
	v1 := fromPoly.verts[(link.edge+1)%fromPoly.vertCount]
	arr := fromTile.data.verts[v0*3 : v0*3+3]
	vCopy(left, arr, 0)
	arr = fromTile.data.verts[v1*3 : v1*3+3]
	vCopy(right, arr, 0)
	// If the link is at tile boundary, dtClamp the vertices to
	// the link width.
	if link.side != 0xff {
		// Unpack portal limits.
		if link.bmin != 0 || link.bmax != 255 {
			s := float32(1.0) / 255.0
			tmin := float32(link.bmin) * s
			tmax := float32(link.bmax) * s
			left = vLerp(fromTile.data.verts, v0*3, v1*3, tmin)
			right = vLerp(fromTile.data.verts, v0*3, v1*3, tmax)
		}
	}
	return &PortalResult{left, right, fromType, toType}
}
func (this *NavMeshQuery) FindNearestPoly(center []float32, extents []float32, filter *QueryFilter) *FindNearestPolyResult {
	nearestPt := []float32{}
	// Get nearby polygons from proximity grid.
	polys := this.queryPolygons(center, extents, filter)
	nearest := int64(0)
	nearestDistanceSqr := FLOAT_MAX_VALUE
	for i := 0; i < len(polys); i++ {
		ref := polys[i]
		closest := this.closestPointOnPoly(ref, center)
		posOverPoly := closest.isPosOverPoly()
		closestPtPoly := closest.getClosest()
		// If a point is directly over a polygon and closer than
		// climb height, favor that instead of straight line nearest point.
		d := float32(0)
		diff := vSub(center, closestPtPoly)
		if posOverPoly {
			first, _ := this.nav.getTileAndPolyByRefUnsafe(polys[i])
			tile := first
			d = float32(math.Abs(float64(diff[1]))) - tile.data.header.walkableClimb
			if d > 0 {
				d = d * d
			} else {
				d = 0
			}
		} else {
			d = vLenSqr(diff)
		}
		if d < nearestDistanceSqr {
			nearestPt = closestPtPoly
			nearestDistanceSqr = d
			nearest = ref
		}
	}
	return &FindNearestPolyResult{nearest, nearestPt}
}
func (this *NavMeshQuery) closestPointOnPoly(ref int64, pos []float32) *ClosesPointOnPolyResult {
	tile, poly := this.nav.getTileAndPolyByRef(ref)
	// Off-mesh connections don't have detail polygons.
	if poly.getType() == DT_POLYTYPE_OFFMESH_CONNECTION {
		v0 := poly.verts[0] * 3
		v1 := poly.verts[1] * 3
		d0 := vDist(pos, tile.data.verts, v0)
		d1 := vDist(pos, tile.data.verts, v1)
		u := d0 / (d0 + d1)
		closest := vLerp(tile.data.verts, v0, v1, u)
		return &ClosesPointOnPolyResult{false, closest}
	}
	// Clamp point to be inside the polygon.
	verts := make([]float32, this.nav.getMaxVertsPerPoly()*3)
	edged := make([]float32, this.nav.getMaxVertsPerPoly())
	edget := make([]float32, this.nav.getMaxVertsPerPoly())
	nv := poly.vertCount
	for i := 0; i < nv; i++ {
		arr := tile.data.verts[poly.verts[i]*3 : poly.verts[i]*3+3]
		vCopy(verts[i*3:i*3+3], arr, 0)
	}
	posOverPoly := false
	closest := make([]float32, 3)
	vCopy(closest, pos, 0)
	if !distancePtPolyEdgesSqr(pos, verts, nv, edged, edget) {
		dmin := edged[0]
		imin := 0
		for i := 1; i < nv; i++ {
			if edged[i] < dmin {
				dmin = edged[i]
				imin = i
			}
		}
		va := imin * 3
		vb := ((imin + 1) % nv) * 3
		closest = vLerp(verts, va, vb, edget[imin])
		posOverPoly = false
	} else {
		posOverPoly = true
	}
	ip := poly.index
	if tile.data.detailMeshes != nil && len(tile.data.detailMeshes) > ip {
		pd := tile.data.detailMeshes[ip]
		// Find height at the location.
		for j := 0; j < pd.triCount; j++ {
			t := (pd.triBase + j) * 4
			v := make([][]float32, 3)
			for k := 0; k < 3; k++ {
				if tile.data.detailTris[t+k] < poly.vertCount {
					index := poly.verts[tile.data.detailTris[t+k]] * 3
					v[k] = []float32{tile.data.verts[index], tile.data.verts[index+1], tile.data.verts[index+2]}
				} else {
					index := (pd.vertBase + (tile.data.detailTris[t+k] - poly.vertCount)) * 3
					v[k] = []float32{tile.data.detailVerts[index], tile.data.detailVerts[index+1], tile.data.detailVerts[index+2]}
				}
			}
			first, second := closestHeightPointTriangle(closest, v[0], v[1], v[2])
			if first {
				closest[1] = second
				break
			}
		}
	}
	return &ClosesPointOnPolyResult{posOverPoly, closest}
}
func (this *NavMeshQuery) queryPolygons(center []float32, extents []float32, filter *QueryFilter) []int64 {
	bmin := vSub(center, extents)
	bmax := vAdd(center, extents)
	// Find tiles the query touches.
	minxy := this.nav.calcTileLoc(bmin)
	minx := minxy[0]
	miny := minxy[1]
	maxxy := this.nav.calcTileLoc(bmax)
	maxx := maxxy[0]
	maxy := maxxy[1]
	polys := []int64{}
	for y := miny; y <= maxy; y++ {
		for x := minx; x <= maxx; x++ {
			neis := this.nav.getTilesAt(x, y)
			for j := 0; j < len(neis); j++ {
				polysInTile := this.queryPolygonsInTile(neis[j], bmin, bmax, filter)
				polys = append(polys, polysInTile...)
			}
		}
	}
	return polys
}
func (this *NavMeshQuery) queryPolygonsInTile(tile *MeshTile, qmin []float32, qmax []float32, filter *QueryFilter) []int64 {
	polys := []int64{}
	if tile.data.bvTree != nil {
		nodeIndex := 0
		tbmin := tile.data.header.bmin
		tbmax := tile.data.header.bmax
		qfac := tile.data.header.bvQuantFactor
		// Calculate quantized box
		bmin := make([]int, 3)
		bmax := make([]int, 3)
		// dtClamp query box to world box.
		minx := clamp_f(qmin[0], tbmin[0], tbmax[0]) - tbmin[0]
		miny := clamp_f(qmin[1], tbmin[1], tbmax[1]) - tbmin[1]
		minz := clamp_f(qmin[2], tbmin[2], tbmax[2]) - tbmin[2]
		maxx := clamp_f(qmax[0], tbmin[0], tbmax[0]) - tbmin[0]
		maxy := clamp_f(qmax[1], tbmin[1], tbmax[1]) - tbmin[1]
		maxz := clamp_f(qmax[2], tbmin[2], tbmax[2]) - tbmin[2]
		// Quantize
		bmin[0] = int(qfac*minx) & 0xfffe
		bmin[1] = int(qfac*miny) & 0xfffe
		bmin[2] = int(qfac*minz) & 0xfffe
		bmax[0] = int(qfac*maxx+1) | 1
		bmax[1] = int(qfac*maxy+1) | 1
		bmax[2] = int(qfac*maxz+1) | 1
		// Traverse tree
		base := this.nav.getPolyRefBase(tile)
		end := tile.data.header.bvNodeCount
		for nodeIndex < end {
			node := tile.data.bvTree[nodeIndex]
			overlap := overlapQuantBounds(bmin, bmax, node.bmin, node.bmax)
			isLeafNode := node.i >= 0
			if isLeafNode && overlap {
				ref := base | int64(node.i)
				if filter.passFilter(ref, tile, tile.data.polys[node.i]) {
					polys = append(polys, ref)
				}
			}
			if overlap || isLeafNode {
				nodeIndex++
			} else {
				nodeIndex += -node.i
			}
		}
		return polys
	} else {
		bmin := make([]float32, 3)
		bmax := make([]float32, 3)
		base := this.nav.getPolyRefBase(tile)
		for i := 0; i < tile.data.header.polyCount; i++ {
			p := tile.data.polys[i]
			if p.getType() == DT_POLYTYPE_OFFMESH_CONNECTION {
				continue
			}
			ref := base | int64(i)
			if !filter.passFilter(ref, tile, p) {
				continue
			}
			// Calc polygon bounds.
			v := p.verts[0] * 3
			vCopy(bmin, tile.data.verts, v)
			vCopy(bmax, tile.data.verts, v)
			for j := 1; j < p.vertCount; j++ {
				v = p.verts[j] * 3
				vMin(bmin, tile.data.verts, v)
				vMax(bmax, tile.data.verts, v)
			}
			if overlapBounds(qmin, qmax, bmin, bmax) {
				polys = append(polys, ref)
			}
		}
		return polys
	}
	return nil
}
