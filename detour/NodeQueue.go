package detour

type NodeQueue struct {
	heap *PriorityQueue
}

func (this *NodeQueue) Init() {
	this.heap = New_PriorityQueue()
}
func (this *NodeQueue) clear() {
	this.heap.Clear()
}
func (this *NodeQueue) top() *Node {
	v := this.heap.Top()
	if v != nil {
		return v.(*Node)
	}
	return nil
}
func (this *NodeQueue) pop() *Node {
	v := this.heap.Pop()
	if v != nil {
		return v.(*Node)
	}
	return nil
}

func (this *NodeQueue) modify(node *Node) {
	this.heap.Remove(node)
	this.heap.Push(node)
}
func (this *NodeQueue) push(node *Node) {
	this.heap.Push(node)
}
func (this *NodeQueue) isEmpty() bool {
	return this.heap.Len() == 0
}
