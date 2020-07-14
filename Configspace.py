import SampleMap
from RobotData import RobotData
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors

# Get Map from the sample map
all_holds = SampleMap.get_map()

l1 = RobotData.l1
l2 = RobotData.l2

pos = np.array([0.47303, 0.43282426])


def dk(base_point, q1, q2):
    x = base_point[0] + l1 * np.cos(q1) + l2 * np.cos(q1 + q2)
    y = base_point[1] + l1 * np.sin(q1) + l2 * np.sin(q1 + q2)

    return np.array([x, y])


def check_collision(ee_position):
    def _sign(p1, p2, p3):
        return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1])

    for h in all_holds:
        o = h.get_hold_plot().get_xydata()
        v1 = o[0]
        v2 = o[1]
        v3 = o[2]

        d1 = _sign(ee_position, v1, v2)
        d2 = _sign(ee_position, v2, v3)
        d3 = _sign(ee_position, v3, v1)

        has_neg = (d1 < 0) or (d2 < 0) or (d3 < 0)
        has_pos = (d1 > 0) or (d2 > 0) or (d3 > 0)

        if not(has_neg and has_pos):
            return True
        else:
            pass
    return False


a1 = np.arange(np.deg2rad(-120), np.deg2rad(120), 0.05)
a2 = np.arange(-np.pi, np.pi, 0.05)

c_space = np.array([])
for q1 in a1:
    for q2 in a2:
        ee_pos = dk(pos, q1, q2)
        if check_collision(ee_pos):
            c_space = np.append(c_space, np.array([q1, q2]))

c_space = c_space.reshape((int(len(c_space) / 2), 2))


###################################### RRT ##########################################

q_init = np.array([-0.83682423, -1.66621619])
q_goal = np.array([0.36197526, -1.45887805])


class Node:
    def __init__(self, pos, parent):
        self.pos = pos
        self.parent = parent


def is_rand_collied(q_rand, c_space):
    r = np.isclose(q_rand, c_space, rtol=0, atol=0.3)
    p = np.array([i.all() for i in r])
    return np.any(p)


def dist(node1, node2):
    return np.linalg.norm(node1.pos - node2.pos)


def closest_node(tree, q_rand):
    distances = np.array([])
    for n in tree:
        distances = np.append(distances, dist(n, Node(q_rand, None)))
    return np.argmin(distances)


def local_planner(q_closest, q_rand, c_space):
    d = dist(Node(q_rand, None), q_closest)
    normalized = np.array([q_rand[0] - q_closest.pos[0], q_rand[1] - q_closest.pos[1]]) / dist(Node(q_rand, None),
                                                                                               q_closest)
    vecs = np.array([])
    for i in range(20):
        v = q_closest.pos + float(i) / 20 * d * normalized
        vecs = np.append(vecs, v)
    vecs = vecs.reshape((int(len(vecs) / 2), 2))

    r = np.array([is_rand_collied(i, c_space) for i in vecs])
    return np.any(r)


K = 1000
delta = 0.2
T1 = [Node(q_init, None)]

for k in range(K):
    q_rand = np.array([np.random.uniform(np.deg2rad(-120), np.deg2rad(120)), np.random.uniform(np.deg2rad(-180), np.deg2rad(180))])
    if not is_rand_collied(q_rand, c_space):
        # Find closest node in tree
        q_closest = T1[closest_node(T1, q_rand)]
        if dist(Node(q_rand, None), q_closest) > delta:
            normalized = np.array([q_rand[0] - q_closest.pos[0], q_rand[1] - q_closest.pos[1]]) / dist(Node(q_rand, None), q_closest)
            q_new = delta * normalized
            q_rand = q_closest.pos + q_new

        if not local_planner(q_closest, q_rand, c_space):
            T1.append(Node(q_rand, q_closest))

# plt.scatter(c_space[:, 0], c_space[:, 1], color='k')
# for t in T1[::-1]:
#     parent = t.parent
#     posx = t.pos[0]
#     posy = t.pos[1]
#     if parent is None:
#         break
#     plt.plot([posx, parent.pos[0]], [posy, parent.pos[1]], 'bo-')
#
# plt.xlim([np.deg2rad(-120), np.deg2rad(120)])
# plt.ylim([-np.pi, np.pi])
# plt.show()


def extend_tree(tree):
    init_length = len(tree)
    while len(tree) != init_length + 1:
        q_rand = np.array([np.random.uniform(np.deg2rad(-120), np.deg2rad(120)), np.random.uniform(np.deg2rad(-180), np.deg2rad(180))])
        if not is_rand_collied(q_rand, c_space):
            # Find closest node in tree
            q_closest = tree[closest_node(tree, q_rand)]
            if dist(Node(q_rand, None), q_closest) > delta:
                normalized = np.array([q_rand[0] - q_closest.pos[0], q_rand[1] - q_closest.pos[1]]) / dist(
                    Node(q_rand, None), q_closest)
                q_new = delta * normalized
                q_rand = q_closest.pos + q_new

            if not local_planner(q_closest, q_rand, c_space):
                tree.append(Node(q_rand, q_closest))
    return tree


def create_internal_nodes(n1, n2):
    dn = n2.pos - n1.pos
    normalized = dn / np.linalg.norm(dn)

    distance = dist(n1, n2)
    internal_nodes = []
    parent_node = n1
    while distance >= delta:
        new_node = Node(parent_node.pos + normalized * delta, parent_node)
        internal_nodes.append(new_node)
        distance = dist(new_node, n2)
        parent_node = new_node

    return internal_nodes


def function():
    T1 = [Node(q_init, None)]
    T2 = [Node(q_goal, None)]

    r1 = []
    r2 = []
    while True:
        extend_tree(T1)
        q_closest = T2[closest_node(T2, np.array([T1[-1].pos[0], T1[-1].pos[1]]))]
        if not local_planner(q_closest, np.array([T1[-1].pos[0], T1[-1].pos[1]]), c_space):
            r_int = create_internal_nodes(T1[-1], q_closest)
            n = T1[-1]
            while n is not None:
                r1.append(n)
                n = n.parent

            n = q_closest
            while n is not None:
                r2.append(n)
                n = n.parent

            r = [r1, r_int, r2]
            return r, T1, T2

        else:
            dT = T1
            T1 = T2
            T2 = dT


r, T1, T2 = function()
a1 = np.array([n.pos for n in r[0][::-1]])
a2 = np.array([n.pos for n in r[1]])
a3 = np.array([n.pos for n in r[2]])
a = np.vstack((a1, a2, a3))

if (a[0] == q_goal).all():
    a = a[::-1]

print(a)
T1 = np.asarray([t.pos for t in T1])
T2 = np.asarray([t.pos for t in T2])


plt.scatter(c_space[:, 0], c_space[:, 1], color='k')
plt.plot(T1[:, 0], T1[:, 1], 'go-')
plt.plot(T2[:, 0], T2[:, 1], 'ro-')
plt.plot(a[:, 0], a[:, 1], 'bo-')
plt.xlim([np.deg2rad(-120), np.deg2rad(120)])
plt.ylim([-np.pi, np.pi])
plt.show()

