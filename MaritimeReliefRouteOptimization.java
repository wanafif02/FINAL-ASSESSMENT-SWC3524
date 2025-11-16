import java.util.*;

public class MaritimeReliefRouteOptimization {
    // Distance Matrix (Adjacency Matrix)
    static int[][] distanceMatrix = {
            {0, 15, 25, 35},
            {15, 0, 30, 28},
            {25, 30, 0, 20},
            {35, 28, 20, 0}
    };
    // Location names
    static String[] locations = {"Port A", "Port B", "Relief Center C", "Relief Center D"};

    // Greedy TSP (Nearest Neighbor)
    public static String greedyTSP(int[][] dist) {
        int n = dist.length;
        boolean[] visited = new boolean[n];
        ArrayList<Integer> route = new ArrayList<>();
        int current = 0; // start at 0 (Port A)
        route.add(current);
        visited[current] = true;
        int total = 0;

        for (int step = 1; step < n; step++) {
            int next = -1;
            int best = Integer.MAX_VALUE;
            for (int j = 0; j < n; j++) {
                if (!visited[j] && dist[current][j] < best) {
                    best = dist[current][j];
                    next = j;
                }
            }
            if (next == -1) break;
            visited[next] = true;
            route.add(next);
            total += dist[current][next];
            current = next;
        }
        // return to start
        total += dist[current][0];
        StringBuilder sb = new StringBuilder();
        sb.append("Greedy TSP Route: ");
        for (int idx = 0; idx < route.size(); idx++) {
            sb.append(locations[route.get(idx)]);
            if (idx < route.size() - 1) sb.append(" -> ");
        }
        sb.append(" -> ").append(locations[0]);
        sb.append(" | Total Distance: ").append(total).append(" nm");
        return sb.toString();
    }

    // Dynamic Programming TSP (Held-Karp)
    public static String dynamicProgrammingTSP(int[][] dist) {
        int n = dist.length;
        int VISITED_ALL = (1 << n) - 1;
        int[][] memo = new int[n][1 << n];
        for (int i = 0; i < n; i++) Arrays.fill(memo[i], -1);
        int[][] next = new int[n][1 << n]; // store next node for path reconstruction
        for (int i = 0; i < n; i++) Arrays.fill(next[i], -1);

        int minCost = dynamicProgrammingTSPHelper(0, 1, dist, memo, VISITED_ALL, next);

        // reconstruct path
        StringBuilder sb = new StringBuilder();
        sb.append("Dynamic Programming TSP Route: ");
        int mask = 1;
        int pos = 0;
        sb.append(locations[pos]);
        while (mask != VISITED_ALL) {
            int nxt = next[pos][mask];
            if (nxt == -1) break; // safety
            sb.append(" -> ").append(locations[nxt]);
            mask |= (1 << nxt);
            pos = nxt;
        }
        sb.append(" -> ").append(locations[0]);
        sb.append(" | Total Distance: ").append(minCost).append(" nm");
        return sb.toString();
    }

    private static int dynamicProgrammingTSPHelper(int pos, int mask, int[][] dist,
                                                   int[][] memo, int VISITED_ALL,
                                                   int[][] next) {
        int n = dist.length;
        if (mask == VISITED_ALL) {
            return dist[pos][0]; // return to start
        }
        if (memo[pos][mask] != -1) return memo[pos][mask];

        int ans = Integer.MAX_VALUE;
        int bestNext = -1;
        for (int city = 0; city < n; city++) {
            if ((mask & (1 << city)) == 0) {
                int newMask = mask | (1 << city);
                int subCost = dynamicProgrammingTSPHelper(city, newMask, dist, memo, VISITED_ALL, next);
                int cost = dist[pos][city] + subCost;
                if (cost < ans) {
                    ans = cost;
                    bestNext = city;
                }
            }
        }
        next[pos][mask] = bestNext;
        memo[pos][mask] = ans;
        return ans;
    }

    // Backtracking TSP (full permutation search)
    public static String backtrackingTSP(int[][] dist) {
        int n = dist.length;
        boolean[] visited = new boolean[n];
        visited[0] = true;
        StringBuilder path = new StringBuilder();
        path.append(locations[0]);
        BacktrackingResult res = new BacktrackingResult();
        tspBacktracking(0, dist, visited, n, 1, 0, path, res);
        // Format result
        StringBuilder sb = new StringBuilder();
        sb.append("Backtracking TSP Route: ").append(res.bestPath);
        sb.append(" | Total Distance: ").append(res.bestCost).append(" nm");
        return sb.toString();
    }

    private static class BacktrackingResult {
        int bestCost = Integer.MAX_VALUE;
        String bestPath = "";
    }

    private static void tspBacktracking(int pos, int[][] dist, boolean[] visited, int n, int count, int cost,
                                       StringBuilder path, BacktrackingResult res) {
        if (count == n) {
            // complete route: return to start
            int totalCost = cost + dist[pos][0];
            StringBuilder fullPath = new StringBuilder(path.toString());
            fullPath.append(" -> ").append(locations[0]);
            if (totalCost < res.bestCost) {
                res.bestCost = totalCost;
                res.bestPath = fullPath.toString();
            }
            return;
        }

        for (int i = 0; i < n; i++) {
            if (!visited[i]) {
                visited[i] = true;
                int added = dist[pos][i];
                int oldLen = path.length();
                path.append(" -> ").append(locations[i]);
                // prune if already worse
                if (cost + added < res.bestCost) {
                    tspBacktracking(i, dist, visited, n, count + 1, cost + added, path, res);
                }
                // backtrack
                path.setLength(oldLen);
                visited[i] = false;
            }
        }
    }

    // Divide and Conquer TSP (for small n, implemented as recursive divide-like search)
    public static String divideAndConquerTSP(int[][] dist) {
        int n = dist.length;
        boolean[] visited = new boolean[n];
        visited[0] = true;
        DivideResult res = new DivideResult();
        StringBuilder path = new StringBuilder();
        path.append(locations[0]);
        divideAndConquerHelper(0, visited, 0, dist, n, path, res);
        StringBuilder sb = new StringBuilder();
        sb.append("Divide & Conquer TSP Route: ").append(res.bestPath);
        sb.append(" | Total Distance: ").append(res.bestCost).append(" nm");
        return sb.toString();
    }

    private static class DivideResult {
        int bestCost = Integer.MAX_VALUE;
        String bestPath = "";
    }

    private static void divideAndConquerHelper(int pos, boolean[] visited, int currentCost, int[][] dist,
                                               int n, StringBuilder path, DivideResult res) {
        // if all visited, close the loop
        if (allVisited(visited)) {
            int total = currentCost + dist[pos][0];
            StringBuilder full = new StringBuilder(path.toString());
            full.append(" -> ").append(locations[0]);
            if (total < res.bestCost) {
                res.bestCost = total;
                res.bestPath = full.toString();
            }
            return;
        }
        // For divide & conquer flavor: choose next half of remaining nodes then the other half (here n small)
        List<Integer> remaining = new ArrayList<>();
        for (int i = 0; i < n; i++) if (!visited[i]) remaining.add(i);

        // Try each remaining node as next (still fundamentally recursive search)
        for (int nxt : remaining) {
            visited[nxt] = true;
            int oldLen = path.length();
            path.append(" -> ").append(locations[nxt]);
            if (currentCost + dist[pos][nxt] < res.bestCost) {
                divideAndConquerHelper(nxt, visited, currentCost + dist[pos][nxt], dist, n, path, res);
            }
            path.setLength(oldLen);
            visited[nxt] = false;
        }
    }

    private static boolean allVisited(boolean[] visited) {
        for (boolean b : visited) if (!b) return false;
        return true;
    }

    // Insertion Sort (sorts in place)
    public static String insertionSort(int[] arr) {
        for (int i = 1; i < arr.length; i++) {
            int key = arr[i];
            int j = i - 1;
            while (j >= 0 && arr[j] > key) {
                arr[j + 1] = arr[j];
                j--;
            }
            arr[j + 1] = key;
        }
        return Arrays.toString(arr);
    }

    // Binary Search (assumes sorted ascending)
    public static String binarySearch(int[] arr, int target) {
        int l = 0, r = arr.length - 1;
        while (l <= r) {
            int mid = l + (r - l) / 2;
            if (arr[mid] == target) return String.valueOf(mid);
            else if (arr[mid] < target) l = mid + 1;
            else r = mid - 1;
        }
        return "-1";
    }

    // Min-Heap (simple wrapper)
    static class MinHeap {
        private PriorityQueue<Integer> pq;

        public MinHeap() {
            pq = new PriorityQueue<>();
        }

        public void insert(int val) {
            pq.add(val);
        }

        public int extractMin() {
            return pq.poll();
        }

        public boolean isEmpty() {
            return pq.isEmpty();
        }
    }

    // Splay Tree implementation (support insert and search)
    static class SplayTree {
        private class Node {
            int key;
            Node left, right, parent;
            Node(int k) { key = k; }
        }

        private Node root = null;

        private void rotateRight(Node x) {
            Node y = x.left;
            if (y == null) return;
            x.left = y.right;
            if (y.right != null) y.right.parent = x;
            y.parent = x.parent;
            if (x.parent == null) root = y;
            else if (x == x.parent.right) x.parent.right = y;
            else x.parent.left = y;
            y.right = x;
            x.parent = y;
        }

        private void rotateLeft(Node x) {
            Node y = x.right;
            if (y == null) return;
            x.right = y.left;
            if (y.left != null) y.left.parent = x;
            y.parent = x.parent;
            if (x.parent == null) root = y;
            else if (x == x.parent.left) x.parent.left = y;
            else x.parent.right = y;
            y.left = x;
            x.parent = y;
        }

        private void splay(Node x) {
            if (x == null) return;
            while (x.parent != null) {
                if (x.parent.parent == null) {
                    // Zig rotation
                    if (x.parent.left == x) rotateRight(x.parent);
                    else rotateLeft(x.parent);
                } else if (x.parent.left == x && x.parent.parent.left == x.parent) {
                    // Zig-Zig
                    rotateRight(x.parent.parent);
                    rotateRight(x.parent);
                } else if (x.parent.right == x && x.parent.parent.right == x.parent) {
                    // Zig-Zig
                    rotateLeft(x.parent.parent);
                    rotateLeft(x.parent);
                } else if (x.parent.left == x && x.parent.parent.right == x.parent) {
                    // Zig-Zag
                    rotateRight(x.parent);
                    rotateLeft(x.parent);
                } else {
                    // Zig-Zag
                    rotateLeft(x.parent);
                    rotateRight(x.parent);
                }
            }
        }

        public void insert(int key) {
            Node z = root;
            Node p = null;
            while (z != null) {
                p = z;
                if (key < z.key) z = z.left;
                else z = z.right;
            }
            z = new Node(key);
            z.parent = p;
            if (p == null) root = z;
            else if (key < p.key) p.left = z;
            else p.right = z;
            splay(z);
        }

        public boolean search(int key) {
            Node z = root;
            while (z != null) {
                if (key == z.key) {
                    splay(z);
                    return true;
                } else if (key < z.key) z = z.left;
                else z = z.right;
            }
            return false;
        }
    }

    // Driver method
    public static void main(String[] args) {
        System.out.println(greedyTSP(distanceMatrix));
        System.out.println(dynamicProgrammingTSP(distanceMatrix));
        System.out.println(backtrackingTSP(distanceMatrix));
        System.out.println(divideAndConquerTSP(distanceMatrix));

        // Sorting and Searching
        int[] arr = {8, 3, 5, 1, 9, 2};
        insertionSort(arr);
        System.out.println("Sorted Array: " + Arrays.toString(arr));
        System.out.println("Binary Search (5 found at index): " + binarySearch(arr, 5));

        // Min-Heap Test
        MinHeap heap = new MinHeap();
        heap.insert(10);
        heap.insert(3);
        heap.insert(15);
        System.out.println("Min-Heap Extract Min: " + heap.extractMin());

        // Splay Tree Test
        SplayTree tree = new SplayTree();
        tree.insert(20);
        tree.insert(10);
        tree.insert(30);
        System.out.println("Splay Tree Search (10 found): " + tree.search(10));
    }
}