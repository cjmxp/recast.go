package recast

const (
	WATERSHED                  int     = 0
	MONOTONE                   int     = 1
	LAYERS                     int     = 2
	RC_NULL_AREA               int     = 0
	LOG_NB_STACKS              int     = 3
	NB_STACKS                  int     = 8 //1 << LOG_NB_STACKS
	MAX_HEIGHT                 int     = 0xffff
	RC_BORDER_REG              int     = 0x8000
	RC_WALKABLE_AREA           int     = 63
	RC_SPAN_HEIGHT_BITS        int     = 13
	RC_SPAN_MAX_HEIGHT         int     = (1 << 13) - 1
	RC_NOT_CONNECTED           int     = 0x3f
	MAX_LAYERS                 int     = RC_NOT_CONNECTED - 1
	RC_CONTOUR_TESS_WALL_EDGES int     = 0x01
	RC_CONTOUR_TESS_AREA_EDGES int     = 0x02
	RC_BORDER_VERTEX           int     = 0x10000
	RC_AREA_BORDER             int     = 0x20000
	RC_CONTOUR_REG_MASK        int     = 0xffff
	RC_MESH_NULL_IDX           int     = 0xffff
	RC_MULTIPLE_REGS           int     = 0
	VERTEX_BUCKET_COUNT        int     = (1 << 12)
	RC_UNSET_HEIGHT            int     = 0xffff
	RETRACT_SIZE               int     = 256
	MAX_VERTS_PER_EDGE         int     = 32
	MAX_VERTS                  int     = 127
	FLOAT_MAX_VALUE            float32 = 3.4028235E38
	EV_UNDEF                   int     = -1
	EV_HULL                    int     = -2
	MAX_TRIS                   int     = 255
)
