import matplotlib.pyplot as plt
import networkx as nx
from networkx.drawing.nx_agraph import graphviz_layout as layout

G = nx.MultiDiGraph()
G.add_edge("u", "v")
pos = nx.spring_layout(G)  # positions for all nodes

# nx.draw_networkx_nodes(G, pos, **options)
# nx.draw_networkx_edges(
#      G,
#      pos,
#      edgelist=[("u", "v")],
#      width=2,
#      alpha=0.5,
#      edge_color="b",
# )
labels = dict()
labels["u"] = r"$\lambda+\beta+\gamma$"
labels["v"] = r"$v$"
# nx.draw_networkx_labels(G, pos, labels, font_size=16)
# plt.axis("off")
# plt.savefig("./test.svg")
# plt.clf()
# plt.show()

options = {"with_labels": True, "node_color": "white", "edgecolors": "blue", "node_size": 12000, "alpha": 0.8, "node_shape": "s"}
nx.set_node_attributes(G, labels, 'labels')

bayes_pos = layout(G, prog="neato")
print(bayes_pos)
ax1 = plt.subplot(1, 3, 1)
nx.draw(G, pos=bayes_pos, labels=labels, **options)
plt.axis("on")
# plt.tight_layout()
plt.xlim((-1000, 1000))
plt.ylim((0, 100))
plt.savefig("./test.svg")
plt.clf()
