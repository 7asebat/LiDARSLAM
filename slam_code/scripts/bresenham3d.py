def Bresenham3D(p, q):
    ListOfPoints = []
    ListOfPoints.append(p)
    dx = abs(q[0] - p[0])
    dy = abs(q[1] - p[1])
    dz = abs(q[2] - p[2])
    if (q[0] > p[0]):
        xs = 1
    else:
        xs = -1
    if (q[1] > p[1]):
        ys = 1
    else:
        ys = -1
    if (q[2] > p[2]):
        zs = 1
    else:
        zs = -1
  
    # Driving axis is X-axis"
    if (dx >= dy and dx >= dz):        
        p1 = 2 * dy - dx
        p2 = 2 * dz - dx
        while (p[0] != q[0]):
            p[0] += xs
            if (p1 >= 0):
                p[1] += ys
                p1 -= 2 * dx
            if (p2 >= 0):
                p[2] += zs
                p2 -= 2 * dx
            p1 += 2 * dy
            p2 += 2 * dz
            ListOfPoints.append([p[0], p[1], p[2]])
  
    # Driving axis is Y-axis"
    elif (dy >= dx and dy >= dz):       
        p1 = 2 * dx - dy
        p2 = 2 * dz - dy
        while (p[1] != q[1]):
            p[1] += ys
            if (p1 >= 0):
                p[0] += xs
                p1 -= 2 * dy
            if (p2 >= 0):
                p[2] += zs
                p2 -= 2 * dy
            p1 += 2 * dx
            p2 += 2 * dz
            ListOfPoints.append([p[0], p[1], p[2]])
  
    # Driving axis is Z-axis"
    else:        
        p1 = 2 * dy - dz
        p2 = 2 * dx - dz
        while (p[2] != q[2]):
            p[2] += zs
            if (p1 >= 0):
                p[1] += ys
                p1 -= 2 * dz
            if (p2 >= 0):
                p[0] += xs
                p2 -= 2 * dz
            p1 += 2 * dy
            p2 += 2 * dx
            ListOfPoints.append([p[0], p[1], p[2]])
    return np.array(ListOfPoints)