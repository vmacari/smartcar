class a_obj (object):

    def __init__(self):
        self.x = 1
        self.__dict__['x'] = 2

    def test(self):



        if 'x' in self.__dict__:
            print('X IN dictionary: {0}'.format(self.x))
        else:
            print('X NOT in dictionary')


def factorial(n):
    """Compute n! recursively.
    :param n: an integer >= 0
    :returns: n!
    Because of Python's stack limitation, this won't
    compute a value larger than about 1000!.
    >>> factorial(5)
    120
    """
    if n == 0: return 1
    return n * factorial(n - 1)

if __name__ == '__main__':
    a_obj().test()
