package detour

import (
	"glog"
	"math"
)

type NavMesh struct {
	params         *NavMeshParams /// < Current initialization params. TODO: do not store this info twice.
	orig           []float32      /// < Origin of the tile (0,0)
	tileWidth      float32
	tileHeight     float32     /// < Dimensions of each tile.
	maxTiles       int         /// < Max number of tiles.
	tileLutSize    int         /// < Tile hash lookup size (must be pot).
	tileLutMask    int         /// < Tile hash lookup mask.
	posLookup      []*MeshTile /// < Tile hash lookup.
	nextFree       *MeshTile   /// < Freelist of tiles.
	tiles          []*MeshTile /// < List of tiles.
	maxVertPerPoly int
	tileCount      int
}

func (this *NavMesh) Init(data *MeshData, maxVertsPerPoly int, flags int) {
	this.params = this.GetNavMeshParams(data)
	this.orig = this.params.orig
	this.tileWidth = this.params.tileWidth
	this.tileHeight = this.params.tileHeight
	// Init tiles
	this.maxTiles = this.params.maxTiles
	this.maxVertPerPoly = maxVertsPerPoly
	lutsize := nextPow2(this.params.maxTiles / 4)
	if lutsize == 0 {
		lutsize = 1
	}
	this.tileLutSize = lutsize
	this.tileLutMask = this.tileLutSize - 1
	this.tiles = make([]*MeshTile, this.maxTiles)
	this.posLookup = make([]*MeshTile, this.tileLutSize)
	this.nextFree = nil
	for i := this.maxTiles - 1; i >= 0; i-- {
		mt := &MeshTile{}
		mt.Init(i)
		this.tiles[i] = mt
		mt.salt = 1
		mt.next = this.nextFree
		this.nextFree = mt
	}
	this.AddTile(data, flags, 0)
}

func (this *NavMesh) InitTile(params *NavMeshParams, maxVertsPerPoly int) {
	this.params = params
	this.orig = params.orig
	this.tileWidth = params.tileWidth
	this.tileHeight = params.tileHeight
	this.maxTiles = params.maxTiles
	this.maxVertPerPoly = maxVertsPerPoly
	lutsize := nextPow2(params.maxTiles / 4)
	if lutsize == 0 {
		lutsize = 1
	}
	this.tileLutSize = lutsize
	this.tileLutMask = this.tileLutSize - 1
	this.tiles = make([]*MeshTile, this.maxTiles)
	this.posLookup = make([]*MeshTile, this.tileLutSize)
	this.nextFree = nil
	for i := this.maxTiles - 1; i >= 0; i-- {
		mt := &MeshTile{}
		mt.Init(i)
		mt.salt = 1
		mt.next = this.nextFree
		this.nextFree = mt
		this.tiles[i] = mt
	}
}

func (this *NavMesh) isValidPolyRef(ref int64) bool {
	if ref == 0 {
		return false
	}
	saltitip := this.decodePolyId(ref)
	salt := saltitip[0]
	it := saltitip[1]
	ip := saltitip[2]
	if it >= this.maxTiles {
		return false
	}
	if this.tiles[it].salt != salt || this.tiles[it].data.header == nil {
		return false
	}

	if ip >= this.tiles[it].data.header.polyCount {
		return false
	}
	return true
}
func (this *NavMesh) GetNavMeshParams(data *MeshData) *NavMeshParams {
	params := &NavMeshParams{}
	params.Init(data.header.bmin, data.header.bmax[0]-data.header.bmin[0], data.header.bmax[2]-data.header.bmin[2], 1, data.header.polyCount)
	return params
}

func (this *NavMesh) AddTile(data *MeshData, flags int, lastRef int64) int64 {
	header := data.header
	if this.getTileAt(header.x, header.y, header.layer) != nil {
		panic("Tile already exists")
	}
	// Allocate a tile.
	var tile *MeshTile = nil
	if lastRef == 0 {
		if this.nextFree != nil {
			tile = this.nextFree
			this.nextFree = tile.next
			tile.next = nil
			this.tileCount++
		}
	} else {
		// Try to relocate the tile to specific index with same salt.
		tileIndex := this.decodePolyIdTile(lastRef)
		if tileIndex >= this.maxTiles {
			panic("Tile index too high")
		}
		// Try to find the specific tile id from the free list.
		target := this.tiles[tileIndex]
		prev := target
		prev = nil
		tile = this.nextFree
		for tile != nil && tile != target {
			prev = tile
			tile = tile.next
		}
		// Could not find the correct location.
		if tile != target {
			panic("Could not find tile")
		}
		// Remove from freelist
		if prev == nil {
			this.nextFree = tile.next
		} else {
			prev.next = tile.next
		}
		// Restore salt.
		tile.salt = this.decodePolyIdSalt(lastRef)
	}
	// Make sure we could allocate a tile.
	if tile == nil {
		panic("Could not allocate a tile")
	}
	tile.data = data
	tile.flags = flags
	tile.links = []*Link{}
	// Insert tile into the position lut.
	h := this.computeTileHash(header.x, header.y, this.tileLutMask)
	tile.next = this.posLookup[h]
	this.posLookup[h] = tile
	// Patch header pointers.

	// If there are no items in the bvtree, reset the tree pointer.
	if tile.data.bvTree != nil && len(tile.data.bvTree) == 0 {
		tile.data.bvTree = nil
	}
	// Init tile.
	this.connectIntLinks(tile)
	// Base off-mesh connections to their starting polygons and connect connections inside the tile.
	this.baseOffMeshLinks(tile)
	this.connectExtOffMeshLinks(tile, tile, -1)
	// Connect with layers in current tile.
	neis := this.getTilesAt(header.x, header.y)
	for j := 0; j < len(neis); j++ {
		if neis[j] == tile {
			continue
		}
		this.connectExtLinks(tile, neis[j], -1)
		this.connectExtLinks(neis[j], tile, -1)
		this.connectExtOffMeshLinks(tile, neis[j], -1)
		this.connectExtOffMeshLinks(neis[j], tile, -1)
	}
	// Connect with neighbour tiles.
	for i := 0; i < 8; i++ {
		neis = this.getNeighbourTilesAt(header.x, header.y, i)
		for j := 0; j < len(neis); j++ {
			this.connectExtLinks(tile, neis[j], i)
			this.connectExtLinks(neis[j], tile, oppositeTile(i))
			this.connectExtOffMeshLinks(tile, neis[j], i)
			this.connectExtOffMeshLinks(neis[j], tile, oppositeTile(i))
		}
	}
	return this.getTileRef(tile)
}

func (this *NavMesh) RemoveTile(ref int64) *MeshData {
	if ref == 0 {
		return nil
	}
	tileIndex := this.decodePolyIdTile(ref)
	tileSalt := this.decodePolyIdSalt(ref)
	if tileIndex >= this.maxTiles {
		panic("Invalid tile index")
	}
	tile := this.tiles[tileIndex]
	if tile.salt != tileSalt {
		panic("Invalid tile salt")
	}
	// Remove tile from hash lookup.
	h := this.computeTileHash(tile.data.header.x, tile.data.header.y, this.tileLutMask)
	cur := this.posLookup[h]
	prev := cur
	prev = nil
	for cur != nil {
		if cur == tile {
			if prev != nil {
				prev.next = cur.next
			} else {
				this.posLookup[h] = cur.next
			}
		}
		prev, cur = cur, cur.next
	}
	// Remove connections to neighbour tiles.
	// Create connections with neighbour tiles.

	// Disconnect from other layers in current tile.
	nneis := this.getTilesAt(tile.data.header.x, tile.data.header.y)
	for i := 0; i < len(nneis); i++ {
		if nneis[i] == tile {
			continue
		}
		this.unconnectLinks(nneis[i], tile)
	}
	// Disconnect from neighbour tiles.
	for i := 0; i < 8; i++ {
		nneis = this.getNeighbourTilesAt(tile.data.header.x, tile.data.header.y, i)
		for i := 0; i < len(nneis); i++ {
			if nneis[i] == tile {
				continue
			}
			this.unconnectLinks(nneis[i], tile)
		}
	}
	data := tile.data
	// Reset tile.
	tile.data = nil
	tile.flags = 0
	tile.links = tile.links[:0]
	tile.salt = (tile.salt + 1) & ((1 << uint8(DT_SALT_BITS)) - 1)
	if tile.salt == 0 {
		tile.salt++
	}
	// Add to free list.
	tile.next = this.nextFree
	this.nextFree = tile
	this.tileCount--
	return data
}

func (this *NavMesh) unconnectLinks(tile, target *MeshTile) {
	if tile == nil || target == nil {
		return
	}
	targetNum := this.decodePolyIdTile(this.getTileRef(target))
	for i := 0; i < tile.data.header.polyCount; i++ {
		poly := tile.data.polys[i]
		j := poly.firstLink
		pj := DT_NULL_LINK
		for j != DT_NULL_LINK {
			if this.decodePolyIdTile(tile.links[j].ref) == targetNum {
				// Remove link.
				nj := tile.links[j].next
				if pj == DT_NULL_LINK {
					poly.firstLink = nj
				} else {
					tile.links[pj].next = nj
				}
				this.freeLink(tile, j)
				j = nj
			} else {
				// Advance
				pj = j
				j = tile.links[j].next
			}
		}
	}
}

func (this *NavMesh) freeLink(tile *MeshTile, link int) {
	tile.links[link].next = tile.linksFreeList
	tile.linksFreeList = link
}

func (this *NavMesh) getTileRef(tile *MeshTile) int64 {
	if tile == nil {
		return 0
	}
	return this.encodePolyId(tile.salt, tile.index, 0)
}
func (this *NavMesh) getNeighbourTilesAt(x, y, side int) []*MeshTile {
	nx, ny := x, y
	switch side {
	case 0:
		nx++
		break
	case 1:
		nx++
		ny++
		break
	case 2:
		ny++
		break
	case 3:
		nx--
		ny++
		break
	case 4:
		nx--
		break
	case 5:
		nx--
		ny--
		break
	case 6:
		ny--
		break
	case 7:
		nx++
		ny--
		break
	}
	return this.getTilesAt(nx, ny)
}

func (this *NavMesh) connectExtLinks(tile *MeshTile, target *MeshTile, side int) {
	if tile == nil {
		return
	}
	// Connect border links.
	for i := 0; i < tile.data.header.polyCount; i++ {
		poly := tile.data.polys[i]
		// Create new links.
		// short m = DT_EXT_LINK | (short)side;
		nv := poly.vertCount
		for j := 0; j < nv; j++ {
			// Skip non-portal edges.
			if (poly.neis[j] & DT_EXT_LINK) == 0 {
				continue
			}
			dir := poly.neis[j] & 0xff
			if side != -1 && dir != side {
				continue
			}
			// Create new links
			va := poly.verts[j] * 3
			vb := poly.verts[(j+1)%nv] * 3
			nei, neia, nnei := this.findConnectingPolys(tile.data.verts, va, vb, target, oppositeTile(dir), 4)
			for k := 0; k < nnei; k++ {
				idx := this.allocLink(tile)
				link := tile.links[idx]
				link.ref = nei[k]
				link.edge = j
				link.side = dir
				link.next = poly.firstLink
				poly.firstLink = idx
				// Compress portal limits to a byte value.
				if dir == 0 || dir == 4 {
					tmin := (neia[k*2+0] - tile.data.verts[va+2]) / (tile.data.verts[vb+2] - tile.data.verts[va+2])
					tmax := (neia[k*2+1] - tile.data.verts[va+2]) / (tile.data.verts[vb+2] - tile.data.verts[va+2])
					if tmin > tmax {
						tmin, tmax = tmax, tmin
					}
					link.bmin = int(clamp_f(tmin, 0, 1) * 255)
					link.bmax = int(clamp_f(tmax, 0, 1) * 255)
				} else if dir == 2 || dir == 6 {
					tmin := (neia[k*2+0] - tile.data.verts[va]) / (tile.data.verts[vb] - tile.data.verts[va])
					tmax := (neia[k*2+1] - tile.data.verts[va]) / (tile.data.verts[vb] - tile.data.verts[va])
					if tmin > tmax {
						tmin, tmax = tmax, tmin
					}
					link.bmin = int(clamp_f(tmin, 0, 1) * 255)
					link.bmax = int(clamp_f(tmax, 0, 1) * 255)
				}
			}
		}
	}
}

func (this *NavMesh) findConnectingPolys(verts []float32, va int, vb int, tile *MeshTile, side int, maxcon int) ([]int64, []float32, int) {
	if tile == nil {
		return nil, nil, 0
	}
	con := make([]int64, maxcon)
	conarea := make([]float32, maxcon*2)
	amin := make([]float32, 2)
	amax := make([]float32, 2)
	this.calcSlabEndPoints(verts, va, vb, amin, amax, side)
	apos := this.getSlabCoord(verts, va, side)
	// Remove links pointing to 'side' and compact the links array.
	bmin := make([]float32, 2)
	bmax := make([]float32, 2)
	m := DT_EXT_LINK | side
	n := 0
	base := this.getPolyRefBase(tile)
	for i := 0; i < tile.data.header.polyCount; i++ {
		poly := tile.data.polys[i]
		nv := poly.vertCount
		for j := 0; j < nv; j++ {
			// Skip edges which do not point to the right side.
			if poly.neis[j] != m {
				continue
			}
			vc := poly.verts[j] * 3
			vd := poly.verts[(j+1)%nv] * 3
			bpos := this.getSlabCoord(tile.data.verts, vc, side)
			// Segments are not close enough.
			if math.Abs(float64(apos-bpos)) > 0.01 {
				continue
			}
			// Check if the segments touch.
			this.calcSlabEndPoints(tile.data.verts, vc, vd, bmin, bmax, side)
			if !this.overlapSlabs(amin, amax, bmin, bmax, 0.01, tile.data.header.walkableClimb) {
				continue
			}
			// Add return value.
			if n < maxcon {
				conarea[n*2+0] = float32(math.Max(float64(amin[0]), float64(bmin[0])))
				conarea[n*2+1] = float32(math.Min(float64(amax[0]), float64(bmax[0])))
				con[n] = base | int64(i)
				n++
			}
			break
		}
	}
	return con, conarea, n
}
func (this *NavMesh) overlapSlabs(amin []float32, amax []float32, bmin []float32, bmax []float32, px float32, py float32) bool {
	// Check for horizontal overlap.
	// The segment is shrunken a little so that slabs which touch
	// at end points are not connected.
	minx := float32(math.Max(float64(amin[0]+px), float64(bmin[0]+px)))
	maxx := float32(math.Min(float64(amax[0]-px), float64(bmax[0]-px)))
	if minx > maxx {
		return false
	}
	// Check vertical overlap.
	ad := (amax[1] - amin[1]) / (amax[0] - amin[0])
	ak := amin[1] - ad*amin[0]
	bd := (bmax[1] - bmin[1]) / (bmax[0] - bmin[0])
	bk := bmin[1] - bd*bmin[0]
	aminy := ad*minx + ak
	amaxy := ad*maxx + ak
	bminy := bd*minx + bk
	bmaxy := bd*maxx + bk
	dmin := bminy - aminy
	dmax := bmaxy - amaxy
	// Crossing segments always overlap.
	if dmin*dmax < 0 {
		return true
	}
	// Check for overlap at endpoints.
	thr := (py * 2) * (py * 2)
	if dmin*dmin <= thr || dmax*dmax <= thr {
		return true
	}
	return false
}
func (this *NavMesh) getSlabCoord(verts []float32, va int, side int) float32 {
	if side == 0 || side == 4 {
		return verts[va]
	} else if side == 2 || side == 6 {
		return verts[va+2]
	}
	return 0
}
func (this *NavMesh) calcSlabEndPoints(verts []float32, va int, vb int, bmin []float32, bmax []float32, side int) {
	if side == 0 || side == 4 {
		if verts[va+2] < verts[vb+2] {
			bmin[0] = verts[va+2]
			bmin[1] = verts[va+1]
			bmax[0] = verts[vb+2]
			bmax[1] = verts[vb+1]
		} else {
			bmin[0] = verts[vb+2]
			bmin[1] = verts[vb+1]
			bmax[0] = verts[va+2]
			bmax[1] = verts[va+1]
		}
	} else if side == 2 || side == 6 {
		if verts[va+0] < verts[vb+0] {
			bmin[0] = verts[va+0]
			bmin[1] = verts[va+1]
			bmax[0] = verts[vb+0]
			bmax[1] = verts[vb+1]
		} else {
			bmin[0] = verts[vb+0]
			bmin[1] = verts[vb+1]
			bmax[0] = verts[va+0]
			bmax[1] = verts[va+1]
		}
	}
}
func (this *NavMesh) connectExtOffMeshLinks(tile *MeshTile, target *MeshTile, side int) {
	if tile == nil {
		return
	}
	// Connect off-mesh links.
	// We are interested on links which land from target tile to this tile.
	oppositeSide := oppositeTile(side)
	if side == -1 {
		oppositeSide = 0xff
	}
	for i := 0; i < target.data.header.offMeshConCount; i++ {
		targetCon := target.data.offMeshCons[i]
		if targetCon.side != oppositeSide {
			continue
		}
		targetPoly := target.data.polys[targetCon.poly]
		// Skip off-mesh connections which start location could not be
		// connected at all.
		if targetPoly.firstLink == DT_NULL_LINK {
			continue
		}
		ext := []float32{targetCon.rad, target.data.header.walkableClimb, targetCon.rad}
		// Find polygon to connect to.
		p := make([]float32, 3)
		p[0] = targetCon.pos[3]
		p[1] = targetCon.pos[4]
		p[2] = targetCon.pos[5]
		nearest := this.findNearestPolyInTile(tile, p, ext)
		ref := nearest.GetNearestRef()
		glog.Println(ref)
		if ref == 0 {
			continue
		}
		nearestPt := nearest.GetNearestPos()
		// findNearestPoly may return too optimistic results, further check
		// to make sure.
		if sqr(nearestPt[0]-p[0])+sqr(nearestPt[2]-p[2]) > sqr(targetCon.rad) {
			continue
		}
		// Make sure the location is on current mesh.
		target.data.verts[targetPoly.verts[1]*3] = nearestPt[0]
		target.data.verts[targetPoly.verts[1]*3+1] = nearestPt[1]
		target.data.verts[targetPoly.verts[1]*3+2] = nearestPt[2]
		// Link off-mesh connection to target poly.
		idx := this.allocLink(target)
		link := target.links[idx]
		link.ref = ref
		link.edge = 1
		link.side = oppositeSide
		link.bmin = link.bmax
		link.bmax = 0
		// Add to linked list.
		link.next = targetPoly.firstLink
		targetPoly.firstLink = idx
		// Link target poly to off-mesh connection.
		if (targetCon.flags & DT_OFFMESH_CON_BIDIR) != 0 {
			tidx := this.allocLink(tile)
			landPolyIdx := this.decodePolyIdPoly(ref)
			landPoly := tile.data.polys[landPolyIdx]
			link = tile.links[tidx]
			link.ref = this.getPolyRefBase(target) | int64(targetCon.poly)
			link.edge = 0xff
			link.side = side
			if side == -1 {
				link.side = 0xff
			}
			link.bmin = link.bmax
			link.bmax = 0
			// Add to linked list.
			link.next = landPoly.firstLink
			landPoly.firstLink = tidx
		}
	}
}

func (this *NavMesh) baseOffMeshLinks(tile *MeshTile) {
	if tile == nil {
		return
	}
	base := this.getPolyRefBase(tile)
	// Base off-mesh connection start points.
	for i := 0; i < tile.data.header.offMeshConCount; i++ {
		con := tile.data.offMeshCons[i]
		poly := tile.data.polys[con.poly]
		ext := []float32{con.rad, tile.data.header.walkableClimb, con.rad}
		// Find polygon to connect to.
		nearestPoly := this.findNearestPolyInTile(tile, con.pos, ext)
		ref := nearestPoly.GetNearestRef()
		if ref == 0 {
			continue
		}
		p := con.pos // First vertex
		nearestPt := nearestPoly.GetNearestPos()
		// findNearestPoly may return too optimistic results, further check
		// to make sure.
		dx := nearestPt[0] - p[0]
		dz := nearestPt[2] - p[2]
		dr := con.rad
		if dx*dx+dz*dz > dr*dr {
			continue
		}
		// Make sure the location is on current mesh.
		arr := nearestPoly.GetNearestPos()[:3]
		copy(tile.data.verts[poly.verts[0]*3:poly.verts[0]*3+3], arr)
		// Link off-mesh connection to target poly.
		idx := this.allocLink(tile)
		link := tile.links[idx]
		link.ref = ref
		link.edge = 0
		link.side = 0xff
		link.bmin = link.bmax
		link.bmax = 0
		// Add to linked list.
		link.next = poly.firstLink
		poly.firstLink = idx
		// Start end-point is always connect back to off-mesh connection.
		tidx := this.allocLink(tile)
		landPolyIdx := this.decodePolyIdPoly(ref)
		landPoly := tile.data.polys[landPolyIdx]
		link = tile.links[tidx]
		link.ref = base | int64(con.poly)
		link.edge = 0xff
		link.side = 0xff
		link.bmin = link.bmax
		link.bmax = 0
		// Add to linked list.
		link.next = landPoly.firstLink
		landPoly.firstLink = tidx
	}
}
func (this *NavMesh) decodePolyIdPoly(ref int64) int {
	polyMask := (int64(1) << DT_POLY_BITS) - 1
	return int(ref & polyMask)
}
func (this *NavMesh) findNearestPolyInTile(tile *MeshTile, center []float32, extents []float32) *FindNearestPolyResult {
	nearestPt := []float32{}
	bmin := vSub(center, extents)
	bmax := vAdd(center, extents)
	// Get nearby polygons from proximity grid.
	polys := this.queryPolygonsInTile(tile, bmin, bmax)
	// Find nearest polygon amongst the nearby polygons.
	nearest := int64(0)
	nearestDistanceSqr := FLOAT_MAX_VALUE
	for i := 0; i < len(polys); i++ {
		ref := polys[i]
		d := float32(0)
		cpp := this.closestPointOnPoly(ref, center)
		posOverPoly := cpp.isPosOverPoly()
		closestPtPoly := cpp.getClosest()
		// If a point is directly over a polygon and closer than
		// climb height, favor that instead of straight line nearest point.
		diff := vSub(center, closestPtPoly)
		if posOverPoly {
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

func (this *NavMesh) closestPointOnPoly(ref int64, pos []float32) *ClosesPointOnPolyResult {
	tile, poly := this.getTileAndPolyByRefUnsafe(ref)
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
	verts := make([]float32, this.maxVertPerPoly*3)
	edged := make([]float32, this.maxVertPerPoly)
	edget := make([]float32, this.maxVertPerPoly)
	nv := poly.vertCount
	for i := 0; i < nv; i++ {
		arr := tile.data.verts[poly.verts[i]*3 : poly.verts[i]*3+3]
		copy(verts[i*3:i*3+3], arr)
	}
	posOverPoly := false
	closest := make([]float32, 3)
	vCopy(closest, pos, 0)
	if !distancePtPolyEdgesSqr(pos, verts, nv, edged, edget) {
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
		posOverPoly = false
	} else {
		posOverPoly = true
	}
	// Find height at the location.
	ip := poly.index
	if tile.data.detailMeshes != nil && len(tile.data.detailMeshes) > ip {
		pd := tile.data.detailMeshes[ip]
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

func (this *NavMesh) queryPolygonsInTile(tile *MeshTile, qmin []float32, qmax []float32) []int64 {
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
		base := this.getPolyRefBase(tile)
		end := tile.data.header.bvNodeCount
		for nodeIndex < end {
			node := tile.data.bvTree[nodeIndex]
			overlap := overlapQuantBounds(bmin, bmax, node.bmin, node.bmax)
			isLeafNode := node.i >= 0
			if isLeafNode && overlap {
				polys = append(polys, base|int64(node.i))
			}
			if overlap || isLeafNode {
				nodeIndex++
			} else {
				escapeIndex := -node.i
				nodeIndex += escapeIndex
			}
		}
		return polys
	} else {
		bmin := make([]float32, 3)
		bmax := make([]float32, 3)
		base := this.getPolyRefBase(tile)
		for i := 0; i < tile.data.header.polyCount; i++ {
			p := tile.data.polys[i]
			// Do not return off-mesh connection polygons.
			if p.getType() == DT_POLYTYPE_OFFMESH_CONNECTION {
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
				polys = append(polys, base|int64(i))
			}
			return polys
		}
	}
	return nil
}

/// Builds internal polygons links for a tile.
func (this *NavMesh) connectIntLinks(tile *MeshTile) {
	if tile == nil {
		return
	}
	base := this.getPolyRefBase(tile)
	for i := 0; i < tile.data.header.polyCount; i++ {
		poly := tile.data.polys[i]
		poly.firstLink = DT_NULL_LINK
		if poly.getType() == DT_POLYTYPE_OFFMESH_CONNECTION {
			continue
		}
		// Build edge links backwards so that the links will be
		// in the linked list from lowest index to highest.
		for j := poly.vertCount - 1; j >= 0; j-- {
			// Skip hard and non-internal edges.
			if poly.neis[j] == 0 || (poly.neis[j]&DT_EXT_LINK) != 0 {
				continue
			}
			idx := this.allocLink(tile)
			link := tile.links[idx]
			link.ref = base | int64((poly.neis[j] - 1))
			link.edge = j
			link.side = 0xff
			link.bmin = link.bmax
			link.bmax = 0
			// Add to linked list.
			link.next = poly.firstLink
			poly.firstLink = idx
		}
	}
}
func (this *NavMesh) allocLink(tile *MeshTile) int {
	if tile.linksFreeList == DT_NULL_LINK {
		link := &Link{}
		link.next = DT_NULL_LINK
		tile.links = append(tile.links, link)
		return len(tile.links) - 1
	}
	link := tile.linksFreeList
	tile.linksFreeList = tile.links[link].next
	return link
}

/// Extracts a tile's salt value from the specified polygon reference.
/// @note This function is generally meant for internal use only.
/// @param[in] ref The polygon reference.
/// @see #encodePolyId
func (this *NavMesh) decodePolyIdSalt(ref int64) int {
	saltMask := int64(1.0<<uint(DT_SALT_BITS)) - 1
	return int((ref >> uint(DT_POLY_BITS+DT_TILE_BITS)) & saltMask)
}

/// Extracts the tile's index from the specified polygon reference.
/// @note This function is generally meant for internal use only.
/// @param[in] ref The polygon reference.
/// @see #encodePolyId
func (this *NavMesh) decodePolyIdTile(ref int64) int {
	tileMask := int64(1.0<<DT_TILE_BITS) - 1
	return int((ref >> DT_POLY_BITS) & tileMask)
}

/**
 * Calculates the tile grid location for the specified world position.
 *
 * @param pos
 *            The world position for the query. [(x, y, z)]
 * @return 2-element int array with (tx,ty) tile location
 */
func (this *NavMesh) calcTileLoc(pos []float32) []int {
	tx := int(math.Floor(float64((pos[0] - this.orig[0]) / this.tileWidth)))
	ty := int(math.Floor(float64((pos[2] - this.orig[2]) / this.tileHeight)))
	return []int{tx, ty}
}
func (this *NavMesh) getTileAt(x int, y int, layer int) *MeshTile {
	h := this.computeTileHash(x, y, this.tileLutMask)
	tile := this.posLookup[h]
	for tile != nil {
		if tile.data.header != nil && tile.data.header.x == x && tile.data.header.y == y && tile.data.header.layer == layer {
			return tile
		}
		tile = tile.next
	}
	return nil
}

func (this *NavMesh) getTilesAt(x int, y int) []*MeshTile {
	tiles := []*MeshTile{}
	// Find tile based on hash.
	h := this.computeTileHash(x, y, this.tileLutMask)
	tile := this.posLookup[h]
	for tile != nil {
		if tile.data.header != nil && tile.data.header.x == x && tile.data.header.y == y {
			tiles = append(tiles, tile)
		}
		tile = tile.next
	}
	return tiles
}
func (this *NavMesh) computeTileHash(x, y, mask int) int {
	h1 := 0x8da6b343 // Large multiplicative constants;
	h2 := 0xd8163841 // here arbitrarily chosen primes
	n := h1*x + h2*y
	return n & mask
}

/**
 * Gets the polygon reference for the tile's base polygon.
 *
 * @param tile
 *            The tile.
 * @return The polygon reference for the base polygon in the specified tile.
 */
func (this *NavMesh) getPolyRefBase(tile *MeshTile) int64 {
	if tile == nil {
		return 0
	}
	return this.encodePolyId(tile.salt, tile.index, 0)
}
func (this *NavMesh) encodePolyId(salt int, it int, ip int) int64 {
	return (int64(salt) << (DT_POLY_BITS + DT_TILE_BITS)) | (int64(it) << DT_POLY_BITS) | int64(ip)
}
func (this *NavMesh) getTileAndPolyByRef(ref int64) (*MeshTile, *Poly) {
	if ref == 0 {
		panic("ref = 0")
	}
	saltitip := this.decodePolyId(ref)
	salt := saltitip[0]
	it := saltitip[1]
	ip := saltitip[2]
	if it >= this.maxTiles {
		panic("tile > m_maxTiles")
	}
	if this.tiles[it].salt != salt || this.tiles[it].data.header == nil {
		panic("Invalid salt or header")
	}
	if ip >= this.tiles[it].data.header.polyCount {
		panic("poly > polyCount")
	}
	return this.tiles[it], this.tiles[it].data.polys[ip]
}
func (this *NavMesh) decodePolyId(ref int64) []int {
	salt, it, ip := 0, 0, 0
	saltMask := (int64(1) << uint(DT_SALT_BITS)) - 1
	tileMask := (int64(1) << uint(DT_TILE_BITS)) - 1
	polyMask := (int64(1) << uint(DT_POLY_BITS)) - 1
	salt = int((ref >> (DT_POLY_BITS + DT_TILE_BITS)) & saltMask)
	it = int((ref >> DT_POLY_BITS) & tileMask)
	ip = int(ref & polyMask)
	return []int{salt, it, ip}
}

func (this *NavMesh) getTileAndPolyByRefUnsafe(ref int64) (*MeshTile, *Poly) {
	saltitip := this.decodePolyId(ref)
	it := saltitip[1]
	ip := saltitip[2]
	return this.tiles[it], this.tiles[it].data.polys[ip]
}

func (this *NavMesh) getMaxVertsPerPoly() int {
	return this.maxVertPerPoly
}
