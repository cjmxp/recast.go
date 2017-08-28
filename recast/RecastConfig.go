package recast

import "math"

type RecastConfig struct {
	PartitionType int

	/** The width/height size of tile's on the xz-plane. [Limit: >= 0] [Units: vx] **/
	TileSize int

	/** The xz-plane cell size to use for fields. [Limit: > 0] [Units: wu] **/
	Cs float32

	/** The y-axis cell size to use for fields. [Limit: > 0] [Units: wu] **/
	Ch float32

	/** The maximum slope that is considered walkable. [Limits: 0 <= value < 90] [Units: Degrees] **/
	WalkableSlopeAngle float32

	/**
	 * Minimum floor to 'ceiling' height that will still allow the floor area to be considered walkable. [Limit: >= 3]
	 * [Units: vx]
	 **/
	WalkableHeight int

	/** Maximum ledge height that is considered to still be traversable. [Limit: >=0] [Units: vx] **/
	WalkableClimb int

	/**
	 * The distance to erode/shrink the walkable area of the heightfield away from obstructions. [Limit: >=0] [Units:
	 * vx]
	 **/
	WalkableRadius int

	/** The maximum allowed length for contour edges along the border of the mesh. [Limit: >=0] [Units: vx] **/
	MaxEdgeLen int

	/**
	 * The maximum distance a simplfied contour's border edges should deviate the original raw contour. [Limit: >=0]
	 * [Units: vx]
	 **/
	MaxSimplificationError float32

	/** The minimum number of cells allowed to form isolated island areas. [Limit: >=0] [Units: vx] **/
	MinRegionArea int

	/**
	 * Any regions with a span count smaller than this value will, if possible, be merged with larger regions. [Limit:
	 * >=0] [Units: vx]
	 **/
	MergeRegionArea int

	/**
	 * The maximum number of vertices allowed for polygons generated during the contour to polygon conversion process.
	 * [Limit: >= 3]
	 **/
	MaxVertsPerPoly int

	/**
	 * Sets the sampling distance to use when generating the detail mesh. (For height detail only.) [Limits: 0 or >=
	 * 0.9] [Units: wu]
	 **/
	DetailSampleDist float32

	/**
	 * The maximum distance the detail mesh surface should deviate from heightfield data. (For height detail only.)
	 * [Limit: >=0] [Units: wu]
	 **/
	DetailSampleMaxError float32
}

func (this *RecastConfig) Init(partitionType int, cellSize float32, cellHeight float32, agentHeight float32, agentRadius float32, agentMaxClimb float32, agentMaxSlope float32, regionMinSize int, regionMergeSize int, edgeMaxLen float32, edgeMaxError float32, vertsPerPoly int, detailSampleDist float32, detailSampleMaxError float32, tileSize int) {
	this.PartitionType = partitionType
	this.Cs = cellSize
	this.Ch = cellHeight
	this.WalkableSlopeAngle = agentMaxSlope
	this.WalkableHeight = int(math.Ceil(float64(agentHeight / cellHeight)))
	this.WalkableClimb = int(math.Floor(float64(agentMaxClimb / cellHeight)))
	this.WalkableRadius = int(math.Ceil(float64(agentRadius / cellSize)))
	this.MaxEdgeLen = int(edgeMaxLen / cellSize)
	this.MaxSimplificationError = edgeMaxError
	this.MinRegionArea = regionMinSize * regionMinSize       // Note: area = size*size
	this.MergeRegionArea = regionMergeSize * regionMergeSize // Note: area = size*size
	this.MaxVertsPerPoly = vertsPerPoly
	this.DetailSampleDist = cellSize * detailSampleDist
	this.DetailSampleMaxError = cellHeight * detailSampleMaxError
	this.TileSize = tileSize
}
