"""Test for the function in the Walker class."""
from round1.walker_helpers import EquiWalkerHelper


def test_equidistance_walk_func1():
    """Test the equidistance walk function of the Walker class.
       this test default movement of the walker
    """

    def_distance_left = 42.0
    def_distance_right = 55.0

    max_left_distance = 200.0
    max_right_distance = 200.0

    walker = EquiWalkerHelper(def_distance_left, def_distance_right,
                               max_left_distance, max_right_distance, 0)

    left_distance = def_distance_left
    right_distance = def_distance_right

    angle = walker.equidistance_walk_func(left_distance,right_distance, 0,current_steering_angle=0)
    assert angle is None

    # Simulate a scenario where the left distance is less than the default
    angle = walker.equidistance_walk_func(left_distance-5, right_distance+5, 0,
                                          current_steering_angle=0)

    assert angle is not None
    assert angle > 0.0  # Expect a positive angle since left distance is less than default

    # Simulate a scenario where the right distance is less than the default
    angle = walker.equidistance_walk_func(left_distance+1, right_distance-1, 0,
                                          current_steering_angle=0)

    angle = walker.equidistance_walk_func(left_distance+2, right_distance-2, 0,
                                          current_steering_angle=0)

    angle = walker.equidistance_walk_func(left_distance+2.5, right_distance-2.5, 0,
                                          current_steering_angle=0)

    assert angle is not None
    assert angle < 0.0  # Expect a negative angle since right distance is less than default
    #find angle datatype.
    print(type(angle))

    assert isinstance(angle, float)

def test_equidistance_walk_func2():
    """Test the equidistance walk function of the Walker class.
       this test if right distance is not set
    """

    def_distance_left = 42.0
    def_distance_right = 200.0

    max_left_distance = 200.0
    max_right_distance = 200.0

    walker = EquiWalkerHelper(def_distance_left, def_distance_right,
                               max_left_distance, max_right_distance, 0)


    angle = walker.equidistance_walk_func(def_distance_left,def_distance_right, 0,
                                           current_steering_angle=0)
    assert angle is None

    # Simulate a scenario where the left distance is less than the default
    angle = walker.equidistance_walk_func(def_distance_left-5, def_distance_right, 0,
                                           current_steering_angle=0)

    assert angle is not None
    assert angle > 0.0  # Expect a positive angle since right distance is increased

    # Simulate a scenario where the right distance is less than the default
    angle = walker.equidistance_walk_func(def_distance_left+1, def_distance_right, 0,
                                           current_steering_angle=0)

    angle = walker.equidistance_walk_func(def_distance_left+2, def_distance_right, 0,
                                           current_steering_angle=0)

    angle = walker.equidistance_walk_func( def_distance_left+2.5, def_distance_right, 0,
                                           current_steering_angle=0)

    assert angle is not None
    assert angle < 0.0  # Expect a negative angle since right distance is less than default

    assert isinstance(angle, float)

def test_equidistance_walk_func3():
    """Test the equidistance walk function of the Walker class.
       this test if left distance is not set
    """

    def_distance_left = 200.0
    def_distance_right = 55.0

    max_left_distance = 200.0
    max_right_distance = 200.0

    walker = EquiWalkerHelper(def_distance_left, def_distance_right,
                               max_left_distance, max_right_distance, 0)


    angle = walker.equidistance_walk_func(def_distance_left,def_distance_right, 0,
                                           current_steering_angle=0)
    assert angle is None

    # Simulate a scenario where the left distance is less than the default
    angle = walker.equidistance_walk_func(def_distance_left, def_distance_right+5, 0,
                                             current_steering_angle=0)

    assert angle is not None
    assert angle > 0.0  # Expect a positive angle since right distance is increased

    # Simulate a scenario where the right distance is less than the default
    angle = walker.equidistance_walk_func(def_distance_left, def_distance_right-1, 0,
                                           current_steering_angle=0)

    angle = walker.equidistance_walk_func(def_distance_left, def_distance_right-2, 0,
                                           current_steering_angle=0)

    angle = walker.equidistance_walk_func(def_distance_left, def_distance_right-2.5, 0,
                                           current_steering_angle=0)

    assert angle is not None
    assert angle < 0.0  # Expect a negative angle since right distance is less than default

    assert isinstance(angle, float)
