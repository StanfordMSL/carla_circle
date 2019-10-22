from utilities import general_tree


def main():
    way1 = general_tree.TreeNode('1', [])
    way2 = general_tree.TreeNode('2', [])
    way3 = general_tree.TreeNode('3', [])
    way4 = general_tree.TreeNode('4', [])
    way5 = general_tree.TreeNode('5', [])
    way6 = general_tree.TreeNode('6', [])
    way7 = general_tree.TreeNode('7', [])
    way8 = general_tree.TreeNode('8', [])
    way9 = general_tree.TreeNode('9', [])
    way10 = general_tree.TreeNode('10', [])
    way11 = general_tree.TreeNode('11', [])
    way12 = general_tree.TreeNode('12', [])
    way13 = general_tree.TreeNode('13', [])
    way14 = general_tree.TreeNode('14', [])
    way15 = general_tree.TreeNode('15', [])
    way16 = general_tree.TreeNode('16', [])
    way17 = general_tree.TreeNode('17', [])
    way18 = general_tree.TreeNode('18', [])
    way19 = general_tree.TreeNode('19', [])
    way20 = general_tree.TreeNode('20', [])
    way21 = general_tree.TreeNode('21', [])
    way22 = general_tree.TreeNode('22', [])
    way23 = general_tree.TreeNode('23', [])
    way24 = general_tree.TreeNode('24', [])

    # Create our test map
    way1.add_child(way2)

    way2.add_child(way3)

    way3.add_child(way4)

    way4.add_child(way5)
    way4.add_child(way11)

    way5.add_child(way6)

    way6.add_child(way7)

    way7.add_child(way8)

    way8.add_child(way9)

    way9.add_child(way10)

    way11.add_child(way12)

    way12.add_child(way13)
    way12.add_child(way17)

    way13.add_child(way14)

    way14.add_child(way15)

    way15.add_child(way16)

    way17.add_child(way18)

    way18.add_child(way19)
    way18.add_child(way21)
    way18.add_child(way23)

    way19.add_child(way20)

    way21.add_child(way22)

    way23.add_child(way24)

    print('data: ', way19.get_data())

    # Create a tree from the map
    root = way1
    tree = general_tree.GeneralTree(root, 'Test Tree')
    tree.print_tree()

    tree.convert_tree_to_arrays()

# Run the test
main()
