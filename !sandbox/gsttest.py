
def mapValues(value, inMin, inMax, outMin, outMax):
    if value < inMin:
        value = inMin

    if value > inMax:
        value = inMax
    # Figure out how 'wide' each range is
    leftSpan = inMax - inMin
    rightSpan = outMax - outMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - inMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return outMin + (valueScaled * rightSpan)


print(mapValues(2200, 1000, 2000, 0, 255))