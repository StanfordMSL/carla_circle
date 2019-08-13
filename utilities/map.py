'''
Utility functions related to a Carla Map object.

'''
import carla
import matplotlib.pyplot as plt
import networkx


def draw_topology(
    carla_map: carla.Map
) -> None:
    '''
    Visualizes the waypoint topology of the provided Carla Map via matplotlib.
    
    Parameters
    ----------
    carla_map : carla.Map
        The Carla Map in which to plot the connected waypoint topology.
    '''
    G = networkx.Graph()
    topology = carla_map.get_topology()
    G.add_edges_from(topology)
    pos = {}

    for node in topology:
        if node[0] not in pos:
            pos[node[0]] = (
                node[0].transform.location.x,
                node[0].transform.location.y
            )

        if node[1] not in pos:
            pos[node[1]] = (
                node[1].transform.location.x,
                node[1].transform.location.y
            )
    
    M = G.number_of_edges()
    edge_colors = range(2, M + 2)
    
    nodes = networkx.draw_networkx_nodes(
        G,
        pos,
        node_size=0.4,
        node_color='blue'
    )
    edges = networkx.draw_networkx_edges(
        G,
        pos,
        node_size=0.2,
        arrowstyle='->',
        arrowsize=10,
        edge_color=edge_colors,
        edge_cmap=plt.cm.Blues,
        width=2
    )
    
    ax = plt.gca()
    ax.set_axis_off()
    plt.show()