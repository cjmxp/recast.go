package recast

import "math"

func FilterLowHangingWalkableObstacles(walkableClimb int, solid *Heightfield) {
	w := solid.width
	h := solid.height
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			previousWalkable := false
			previousArea := RC_NULL_AREA
			var ps *Span = nil
			for s := solid.spans[x+y*w]; s != nil; ps, s = s, s.next {
				walkable := s.area != RC_NULL_AREA
				// If current span is not walkable, but there is walkable
				// span just below it, mark the span above it walkable too.
				if !walkable && previousWalkable {
					if int(math.Abs(float64(s.smax-ps.smax))) <= walkableClimb {
						s.area = previousArea
					}
				}
				// Copy walkable flag so that it cannot propagate
				// past multiple non-walkable objects.
				previousWalkable = walkable
				previousArea = s.area
			}
		}
	}
}

func FilterLedgeSpans(walkableHeight int, walkableClimb int, solid *Heightfield) {
	w := solid.width
	h := solid.height
	// Mark border spans.
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			for s := solid.spans[x+y*w]; s != nil; s = s.next {
				// Skip non walkable spans.
				if s.area == RC_NULL_AREA {
					continue
				}
				bot := s.smax
				top := MAX_HEIGHT
				if s.next != nil {
					top = s.next.smin
				}
				// Find neighbours minimum height.
				minh := MAX_HEIGHT
				// Min and max height of accessible neighbours.
				asmin := s.smax
				asmax := s.smax
				for dir := 0; dir < 4; dir++ {
					dx := x + GetDirOffsetX(dir)
					dy := y + GetDirOffsetY(dir)
					// Skip neighbours which are out of bounds.
					if dx < 0 || dy < 0 || dx >= w || dy >= h {
						minh = int(math.Min(float64(minh), float64(-walkableClimb-bot)))
						continue
					}
					// From minus infinity to the first span.
					ns := solid.spans[dx+dy*w]
					nbot := -walkableClimb
					ntop := MAX_HEIGHT
					if ns != nil {
						ntop = ns.smin
					}
					// Skip neightbour if the gap between the spans is too small.
					if int(math.Min(float64(top), float64(ntop))-math.Max(float64(bot), float64(nbot))) > walkableHeight {
						minh = int(math.Min(float64(minh), float64(nbot-bot)))
					}
					// Rest of the spans.
					for ns = solid.spans[dx+dy*w]; ns != nil; ns = ns.next {
						nbot = ns.smax
						ntop = MAX_HEIGHT
						if ns.next != nil {
							ntop = ns.next.smin
						}
						// Skip neightbour if the gap between the spans is too small.
						if int(math.Min(float64(top), float64(ntop))-math.Max(float64(bot), float64(nbot))) > walkableHeight {
							minh = int(math.Min(float64(minh), float64(nbot-bot)))
							// Find min/max accessible neighbour height.
							if int(math.Abs(float64(nbot-bot))) <= walkableClimb {
								if nbot < asmin {
									asmin = nbot
								}
								if nbot > asmax {
									asmax = nbot
								}
							}
						}
					}
				}
				// The current span is close to a ledge if the drop to any
				// neighbour span is less than the walkableClimb.
				if minh < -walkableClimb {
					s.area = RC_NULL_AREA
				}
				// If the difference between all neighbours is too large,
				// we are at steep slope, mark the span as ledge.
				if (asmax - asmin) > walkableClimb {
					s.area = RC_NULL_AREA
				}
			}
		}
	}
}

func FilterWalkableLowHeightSpans(walkableHeight int, solid *Heightfield) {
	w := solid.width
	h := solid.height
	// Remove walkable flag from spans which do not have enough
	// space above them for the agent to stand there.
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			for s := solid.spans[x+y*w]; s != nil; s = s.next {
				bot := s.smax
				top := MAX_HEIGHT
				if s.next != nil {
					top = s.next.smin
				}
				if (top - bot) <= walkableHeight {
					s.area = RC_NULL_AREA
				}
			}
		}
	}
}
