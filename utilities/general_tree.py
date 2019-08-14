from __future__ import print_function


class TreeNode(object):
    def __init__(self, data, children):
        self.data = data
        self.children = children

    def add_child(self, obj):
        self.children.append(obj)

    def print_node(self, depth = 0):
        if depth > 0:
            print_character_repeat(depth - 1, '|  ')
            print('|->', end='')

        print(self.data)

        if self.children:
            for child in self.children:
                child.print_node(depth + 1)

    def has_children(self):
        if self.children:
            return True
        else:
            return False

    def get_children(self):
        return self.children

    def get_child(self, index):
        return self.children[index]

    def get_data(self):
        return self.data

    def get_num_children(self):
        return len(self.children)

    def set_data(self, data):
        self.data = data

    def set_children(self, children):
        self.children = children


class GeneralTree(object):
    def __init__(
        self,
        root: TreeNode,
        name: str = 'Default Tree'
    ):
        self.root = root
        self.name = name

    def convert_tree_to_arrays(self):
        arrays = []
        create_path(self.root, [], 0)
        print(arrays)

        return arrays

    def print_leaf(self, node):
        if node:
            node.print_node()

    def print_tree(self):
        if self.root:
            print('')
            print(self.name)
            print_character_repeat(len(self.name), '-')
            print('')
            self.root.print_node()

        print('')

    def get_root(self):
        return self.root

    def set_root(self, root):
        self.root = root


def print_character_repeat(num, char = ' '):
    for i in range(num):
        print(char, end = '')


def create_path(root, path, path_length):
    if root is None:
        return

    if len(path) > path_length:
        path[path_length] = root.get_data()
    else:
        path.append(root.get_data())

    path_length = path_length + 1

    if root.has_children():
        for child in root.get_children():
            create_path(child, path, path_length)
    else:
        print(path)
        return path
