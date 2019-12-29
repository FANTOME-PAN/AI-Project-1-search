
def permutation(elem_lst):
    res = []
    if len(elem_lst) == 0:
        return res
    if len(elem_lst) == 1:
        res.append([elem_lst[0]])
        return res
    for i in range(len(elem_lst)):
        prefix = elem_lst[i]
        elem_left = [j for j in elem_lst if j != prefix]
        postfixes = permutation(elem_left)
        for postfix in postfixes:
            postfix.insert(0, prefix)
            res.append(postfix)
    return res


def __main__():
    res = permutation([1, 2, 3])
    print res


__main__()