def save_to_joints(emotion_level):
    names = list()
    times = list()
    keys = list()

    names.append("HeadPitch")
    times.append([0.16, 0.68, 1.16, 1.68, 2.16, 2.68, 3.16, 3.68])
    keys.append([0.00149202, 0.0797259, 0.254602, 0.389594, 0.308291, 0.115008, 0.0183661, 0.0199001])

    names.append("HeadYaw")
    times.append([0.16, 0.68, 1.16, 1.68, 2.16, 2.68, 3.16, 3.68])
    keys.append([-0.01845, 0.0260359, 0.236194, 0.283749, 0.28068, 0.11961, 0.00609397, 0.00149202])

    names.append("LElbowRoll")
    times.append([0.16, 0.68, 1.16, 1.68, 2.16, 2.68, 3.16, 3.68])
    keys.append([-0.650374, -0.674919, -0.782298, -0.946436, -0.912689, -0.905018, -0.854396, -0.828318])

    names.append("LElbowYaw")
    times.append([0.16, 0.68, 1.16, 1.68, 2.16, 2.68, 3.16, 3.68])
    keys.append([-0.1335, -0.113558, -0.219404, -0.289967, -0.286901, -0.174919, -0.066004, -0.038392])

    names.append("LHand")
    times.append([0.16, 0.68, 1.16, 1.68, 2.16, 2.68, 3.16, 3.68])
    keys.append([0.0208, 0.0208, 0.0208, 0.0208, 0.0212001, 0.0208, 0.0208, 0.0208])

    names.append("LShoulderPitch")
    times.append([0.16, 0.68, 1.16, 1.68, 2.16, 2.68, 3.16, 3.68])
    keys.append([1.01393, 0.553732, -0.358999, -0.444902, -0.329852, 0.423342, 0.85593, 0.972515])

    names.append("LShoulderRoll")
    times.append([0.16, 0.68, 1.16, 1.68, 2.16, 2.68, 3.16, 3.68])
    keys.append([0.110406, 0.056716, -0.093616, -0.191792, -0.162646, 0.033706, 0.0459781, 0.0398422])

    names.append("LWristYaw")
    times.append([0.16, 0.68, 1.16, 1.68, 2.16, 2.68, 3.16, 3.68])
    keys.append([-0.871354, -0.871354, -0.871354, -0.871354, -0.871354, -0.871354, -0.871354, -0.871354])

    return names, times, keys