def save_to_joints(emotion_level):
    names = list()
    times = list()
    keys = list()

    names.append("HeadPitch")
    times.append([0.76, 2.24, 3.76])
    keys.append([-0.0598679, 0.51354, -0.036858])

    names.append("HeadYaw")
    times.append([0.76, 2.24, 3.76])
    keys.append([0.00762796, 0.00762796, 0.00455999])

    names.append("LElbowRoll")
    times.append([0.24, 0.76, 1.24, 1.76, 2.24, 2.76, 3.24, 3.76])
    keys.append([-0.835988, -0.854396, -1.54462, -1.54462, -1.54462, -1.40664, -1.1704, -1.03234])

    names.append("LElbowYaw")
    times.append([0.24, 0.76, 1.24, 1.76, 2.24, 2.76, 3.24, 3.76])
    keys.append([0.139552, 0.061318, -0.894364, -1.02782, -1.03242, -0.914306, -0.170316, 0.101202])

    names.append("LHand")
    times.append([0.24, 0.76, 1.24, 1.76, 2.24, 2.76, 3.24, 3.76])
    keys.append([0.0208, 0.0208, 0.0208, 0.0208, 0.0208, 0.0208, 0.0208, 0.0208])

    names.append("LShoulderPitch")
    times.append([0.24, 0.76, 1.24, 1.76, 2.24, 2.76, 3.24, 3.76])
    keys.append([0.944902, 0.932631, 0.674919, 0.352778, 0.348176, 0.50311, 0.808375, 0.891212])

    names.append("LShoulderRoll")
    times.append([0.24, 0.76, 1.24, 1.76, 2.24, 2.76, 3.24, 3.76])
    keys.append([0.11194, 0.0628521, 0.00302602, -0.177985, -0.159578, -0.0261199, 0.164096, 0.131882])

    names.append("LWristYaw")
    times.append([0.24, 0.76, 1.24, 1.76, 2.24, 2.76, 3.24, 3.76])
    keys.append([-1.40979, -1.40672, -1.40672, -1.40672, -1.40672, -1.40519, -1.40365, -1.40365])

    names.append("RElbowRoll")
    times.append([0.24, 0.76, 1.24, 1.76, 2.24, 2.76, 3.24, 3.76])
    keys.append([0.727158, 0.753235, 1.4374, 1.51563, 1.50336, 1.30854, 1.13674, 0.955723])

    names.append("RElbowYaw")
    times.append([0.24, 0.76, 1.24, 1.76, 2.24, 2.76, 3.24, 3.76])
    keys.append([0.0689882, 0.115008, 0.843657, 1.07376, 1.05995, 0.921892, 0.325165, 0.0889301])

    names.append("RHand")
    times.append([0.24, 0.76, 1.24, 1.76, 2.24, 2.76, 3.24, 3.76])
    keys.append([0.2732, 0.2732, 0.2732, 0.2732, 0.2732, 0.2732, 0.2732, 0.2732])

    names.append("RShoulderPitch")
    times.append([0.24, 0.76, 1.24, 1.76, 2.24, 2.76, 3.24, 3.76])
    keys.append([1.04776, 1.04316, 0.707216, 0.446436, 0.440299, 0.541544, 0.874422, 1.01402])

    names.append("RShoulderRoll")
    times.append([0.24, 0.76, 1.24, 1.76, 2.24, 2.76, 3.24, 3.76])
    keys.append([-0.00464392, 0.030638, 0.139552, 0.314159, 0.314159, 0.21932, -0.0813439, -0.0828778])

    names.append("RWristYaw")
    times.append([0.24, 0.76, 1.24, 1.76, 2.24, 2.76, 3.24, 3.76])
    keys.append([0.670316, 0.671851, 0.674919, 0.676453, 0.676453, 0.671851, 0.619695, 0.619695])

    return names, times, keys
