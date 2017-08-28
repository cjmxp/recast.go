package detour

type NodePool struct {
	_map  map[int64][]*Node
	nodes []*Node
}

func (this *NodePool) Init() {
	this._map = make(map[int64][]*Node)
	this.nodes = []*Node{}
}

func (this *NodePool) getNode(ref int64) *Node {
	return this.getNode2(ref, 0)
}
func (this *NodePool) getNode2(id int64, state int) *Node {
	if nodes, ok := this._map[id]; ok {
		for i := 0; i < len(nodes); i++ {
			node := nodes[i]
			if node.state == state {
				return node
			}
		}
	}
	return this.create(id, state)
}
func (this *NodePool) getNodeIdx(node *Node) int {
	if node != nil {
		return node.index
	}
	return 0
}
func (this *NodePool) getNodeAtIdx(idx int) *Node {
	if idx != 0 {
		return this.nodes[idx-1]
	}
	return nil
}
func (this *NodePool) create(id int64, state int) *Node {
	node := &Node{}
	node.Init(len(this.nodes) + 1)
	node.id = id
	node.state = state
	this.nodes = append(this.nodes, node)
	if _, ok := this._map[id]; !ok {
		this._map[id] = []*Node{}
	}
	this._map[id] = append(this._map[id], node)
	return node
}

func (this *NodePool) clear() {
	this._map = make(map[int64][]*Node)
	this.nodes = this.nodes[:0]
}
