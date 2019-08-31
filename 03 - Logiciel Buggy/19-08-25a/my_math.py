# math tool
def constraint(variable, min, max):
    if variable>max:
        return max
    elif variable<min:
        return min
    else:
        return variable
