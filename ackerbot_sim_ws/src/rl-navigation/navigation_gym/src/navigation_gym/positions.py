import numpy as np


def pos_env_train():
    env = np.random.randint(0, 2)
    p_x, p_y, p_z = 0.0, 0.0, 0.05
    o_x, o_y, o_z, o_w = 0.0, 0.0, 0.75, 0.75
    if env == 0:
        choice = np.random.randint(0, 2)
        if choice == 0:
            p_x = np.random.uniform(-0.5, 5.5)
            p_y = np.random.uniform(-0.5, 2.)
            t_x = np.random.uniform(-4.25, -3.75)
            t_y = np.random.uniform(-5.25, -4.75)
            o_w = np.random.uniform(3.8, 4.2)
        else:
            p_x = np.random.uniform(-4.25, -3.75)
            p_y = np.random.uniform(-5.25, -4.75)
            t_x = np.random.uniform(-0.5, 5.5)
            t_y = np.random.uniform(-0.5, 2.)
            o_w = np.random.uniform(1.3, 1.7)
        ini_pos = {'p_x': p_x, 'p_y': p_y, 'p_z': p_z, 'o_x': o_x,
                   'o_y': o_y, 'o_z': np.random.uniform(1.3, 1.7), 'o_w': o_w}
        target_pos = (t_x, t_y)
        return ini_pos, target_pos

    elif env == 1:
        choice = np.random.randint(0, 2)
        if choice == 0:
            p_x = np.random.uniform(12., 16.5)
            p_y = np.random.uniform(1.75, 2.25)
            t_x = np.random.uniform(12., 16.5)
            t_y = np.random.uniform(-5.75, -6.25)
            o_z, o_w = np.random.uniform(1.3, 1.7), np.random.uniform(-1.3, -1.7)
        else:
            p_x = np.random.uniform(12., 16.5)
            p_y = np.random.uniform(-5.75, -6.25)
            t_x = np.random.uniform(12.0, 16.5)
            t_y = np.random.uniform(.75, 2.25)
            o_z, o_w = np.random.uniform(3.2, 3.6), np.random.uniform(1.3, 1.7)

        ini_pos = {'p_x': p_x, 'p_y': p_y, 'p_z': p_z, 'o_x': o_x,
                   'o_y': o_y, 'o_z': o_z, 'o_w': o_w}
        target_pos = (t_x, t_y)
        return ini_pos, target_pos


def pos_env_val(direction):
    p_x, p_y, p_z = 0.0, 0.0, 0.05
    o_x, o_y = 0.0, 0.0
    if direction == 'forward':
        p_x = np.random.uniform(-3.4, -3.6)
        p_y = np.random.uniform(-9.4, -9.6)
        t_x = -1.0
        t_y = -6.5
    else:
        p_x = np.random.uniform(-0.9, -1.1)
        p_y = np.random.uniform(-6.4, -6.6)
        t_x = -3.5
        t_y = -9.5

    o_z = 0.
    o_w = -1.5
    ini_pos = {'p_x': p_x, 'p_y': p_y, 'p_z': p_z, 'o_x': o_x,
               'o_y': o_y, 'o_z': o_z, 'o_w': o_w}
    target_pos = (t_x, t_y)
    return ini_pos, target_pos


def pos_custom(starting_pos, start_orientation, target_pos):
    p_x, p_y, p_z = 0.0, 0.0, 0.05
    o_x, o_y, o_z, o_w = 0.0, 0.0, 0.75, 0.75
    p_x, p_y = starting_pos[0] + np.random.uniform(-0.2, 0.2), starting_pos[1] + np.random.uniform(-0.2, 0.2)
    t_x, t_y = target_pos[0], target_pos[1]
    o_z, o_w = start_orientation[0], start_orientation[1]
    ini_pos = {'p_x': p_x, 'p_y': p_y, 'p_z': p_z, 'o_x': o_x,
               'o_y': o_y, 'o_z': o_z, 'o_w': o_w}
    target_pos = (t_x, t_y)
    return ini_pos, target_pos
