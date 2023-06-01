
import networkx as nx
import matplotlib.pyplot as plt

# Load the GraphML file
G = nx.read_graphml('graph_file.graphml')

# Create a new Graph containing only nodes with 'key2' attribute as 'File'
G_filtered = nx.DiGraph()  # DiGraph to ensure the graph is treated as a tree
for node, data in G.nodes(data=True):
        G_filtered.add_node(node, **data)

# Add edges between 'File' nodes
for u, v, data in G.edges(data=True):
        G_filtered.add_edge(u, v, **data)

# Create a mapping from node ID to 'key1' value for renaming the nodes
id_to_key1 = {id: data.get('key1', id) for id, data in G_filtered.nodes(data=True)}

# Draw the graph
pos = nx.spring_layout(G_filtered)  # positions for all nodes

# nodes
nx.draw_networkx_nodes(G_filtered, pos, node_size=700)

# edges
nx.draw_networkx_edges(G_filtered, pos, width=1)

# labels
nx.draw_networkx_labels(G_filtered, pos, labels=id_to_key1, font_size=10, font_family='sans-serif')

plt.axis('off')
plt.show()

