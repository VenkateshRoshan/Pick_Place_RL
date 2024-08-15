import pybullet as p

def create_marker_point(x, y, z, size, colour=[0, 255, 0, 255]):
    """
    Create a marker point in the PyBullet simulation environment.
    """
    shape_id = p.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=size,
        rgbaColor=colour,
    )

    p.createMultiBody(
        baseMass=0,
        baseInertialFramePosition=[0, 0, 0],
        baseVisualShapeIndex=shape_id,
        basePosition=[x, y, z],
    )
    return shape_id