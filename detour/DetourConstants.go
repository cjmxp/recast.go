package detour

const (
	XP                             int     = 1 << 0
	ZP                             int     = 1 << 1
	XM                             int     = 1 << 2
	ZM                             int     = 1 << 3
	FLOAT_MAX_VALUE                float32 = 3.4028235E38
	MESH_NULL_IDX                  int     = 0xffff
	DT_NAVMESH_MAGIC               int     = 'D'<<24 | 'N'<<16 | 'A'<<8 | 'V' /** A magic number used to detect compatibility of navigation tile data. */
	DT_NAVMESH_VERSION             int     = 7                                /** A version number used to detect compatibility of navigation tile data.*/
	DT_NAVMESH_VERSION_RECAST4J    int     = 0x8807
	DT_NAVMESH_STATE_MAGIC         int     = 'D'<<24 | 'N'<<16 | 'M'<<8 | 'S' /** A magic number used to detect the compatibility of navigation tile states.*/
	DT_NAVMESH_STATE_VERSION       int     = 1                                /** A version number used to detect compatibility of navigation tile states.*/
	DT_POLYTYPE_GROUND             int     = 0                                /** The polygon is a standard convex polygon that is part of the surface of the mesh. */
	DT_POLYTYPE_OFFMESH_CONNECTION int     = 1                                /** The polygon is an off-mesh connection consisting of two vertices. */
	DT_EXT_LINK                    int     = 0x8000
	DT_OFFMESH_CON_BIDIR           int     = 1 /// A flag that indicates that an off-mesh connection can be traversed in  both directions. (Is bidirectional.)
	DT_NULL_LINK                   int     = -1
	DT_MAX_AREAS                   int     = 64 /// The maximum number of user defined area ids.
	DT_SALT_BITS                   int     = 16
	DT_POLY_BITS                   uint8   = 20
	DT_TILE_BITS                   uint8   = 28
	EPS                            float32 = 1.0E-4

	DT_NODE_OPEN                       int     = 0x01
	DT_NODE_CLOSED                     int     = 0x02
	FAILURE                            int     = 0
	SUCCSESS                           int     = 1
	IN_PROGRESS                        int     = 2
	PARTIAL_RESULT                     int     = 3
	H_SCALE                            float32 = 0.999 // Search heuristic scale.
	DT_STRAIGHTPATH_START              int     = 0x01  /** The vertex is the start position in the path. */
	DT_STRAIGHTPATH_END                int     = 0x02  /** The vertex is the end position in the path. */
	DT_STRAIGHTPATH_AREA_CROSSINGS     int     = 0x01  ///< Add a vertex at every polygon edge crossing where area changes.
	DT_STRAIGHTPATH_ALL_CROSSINGS      int     = 0x02  ///< Add a vertex at every polygon edge crossing.
	DT_STRAIGHTPATH_OFFMESH_CONNECTION int     = 0x04  /** The vertex is the start of an off-mesh connection. */
)
