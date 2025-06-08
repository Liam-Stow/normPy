from typing import Optional

MaybeNode = Optional['LinearInterpolationMap.Node']

class LinearInterpolationMap:
    """
    A map for linear interpolation between (x, y) pairs.

    This class allows you to create a map from a set of (x, y) data points, and interpolate
    the y-value for any given x using linear interpolation. Use insert(x, y) to add new (x, y) pairs.
    The data is stored in a balanced binary search tree for fast lookups and insertions. 
    Out-of-bounds queries return the nearest y-value.

    Methods:
        insert(x, y):
            Insert a new (x, y) pair into the map.
        interpolate(x):
            Return the interpolated y-value for a given x.
    """
    class Node:
        """
        Node for the binary search tree, representing a single (x, y) pair.
        """
        def __init__(self, x: float, y: float, left: MaybeNode = None, right: MaybeNode = None):
            self.x = x
            self.y = y
            self.left = left
            self.right = right

    def __init__(self):
        self.x_max: float = float('-inf')
        self.x_min: float = float('inf')
        self.y_max: float = float('-inf')
        self.y_min: float = float('inf')
        self.root: MaybeNode = None

    def _build_bst(self, pairs: list[tuple[float, float]]) -> MaybeNode:
        """Build a balanced binary search tree from sorted (x, y) pairs."""
        if not pairs:
            return None
        mid = len(pairs) // 2
        x, y = pairs[mid]
        left = self._build_bst(pairs[:mid])
        right = self._build_bst(pairs[mid+1:])
        return self.Node(x, y, left, right)

    def interpolate(self, x: float) -> float | None:
        """Interpolate the y-value for a given x using linear interpolation."""
        # Handle out-of-bounds
        if self.root is None:
            return None
        if x <= self.x_min:
            return self.y_min
        if x >= self.x_max:
            return self.y_max
        
        # Find the two nodes between which x lies
        lower, upper = self._find_bounds(self.root, x, None, None)
        if lower is None or upper is None:
            return None

        # Linear interpolation
        x0, y0 = lower.x, lower.y
        x1, y1 = upper.x, upper.y
        slope = (y1 - y0) / (x1 - x0)
        return y0 + slope * (x - x0)

    def _find_bounds(self, node: MaybeNode, x: float, lower: MaybeNode, upper: MaybeNode) -> tuple[MaybeNode, MaybeNode]:
        """Find the two nodes in the BST that bound the given x value."""
        if node is None:
            return lower, upper
        if x < node.x:
            return self._find_bounds(node.left, x, lower, node)
        elif x > node.x:
            return self._find_bounds(node.right, x, node, upper)
        else:
            # Exact match
            return node, node

    def insert(self, x: float, y: float) -> None:
        """Insert a new (x, y) pair into the BST, maintaining the BST property."""
        def _insert(node: Optional['LinearInterpolationMap.Node'], x: float, y: float) -> 'LinearInterpolationMap.Node':
            if node is None:
                return self.Node(x, y)
            if x < node.x:
                node.left = _insert(node.left, x, y)
            elif x > node.x:
                node.right = _insert(node.right, x, y)
            else:
                node.y = y  # Update y if x already exists
            return node
        self.root = _insert(self.root, x, y)
        
        # Update min/max values
        self.x_min = min(self.x_min, x)
        self.x_max = max(self.x_max, x)
        self.y_min = min(self.y_min, y)
        self.y_max = max(self.y_max, y)

