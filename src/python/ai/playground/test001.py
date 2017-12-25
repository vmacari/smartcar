
def binary_search(iarray, value):
    lo = 0
    hi  = len(iarray) -1

    while lo <= hi:
        mid = (lo + hi) //2
        if value == iarray[mid]:
            return mid
        elif value <  iarray[mid]:
            hi = mid -1
        else:
            lo = mid + 1
    return None

if __name__ == '__main__':

    print(binary_search([1, 2, 3, 4, 5], 4))
