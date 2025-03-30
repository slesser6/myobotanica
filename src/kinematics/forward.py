from spatialmath import SE3

def solve_fk(joints):
    test = SE3.Trans(0.6, -0.3, 0.1) * SE3.OA([0, 1, 0], [0, 0, -1]) # test
    print(test)
    return "FK solution"