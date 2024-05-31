import math

# imagine a ball thrown with initial degree with a given angular and linear velocity
# (ignore gravity, we will use angular velocity and linear velocity instead)
# We will calculate the travel distance when it lands

def run(initial_degree, angular_vel, linear_vel):
    initial_degree = math.radians(initial_degree)
    landing_degree = -initial_degree  # should always be like this I think
    total_degree_travelled = initial_degree - landing_degree

    angular_velocity = math.radians(angular_vel)
    velocity = linear_vel
    duration = total_degree_travelled / angular_velocity


    time_steps_per_second = 300  # the higher the more precise the calculation will be

    n = round(duration * time_steps_per_second)

    total_x = 0

    for i in range(n):
        current_second = i/n * duration
        dt = 1 / time_steps_per_second
        current_angle = initial_degree - angular_velocity*current_second
        dx = velocity * dt * math.cos(current_angle)
        total_x += dx

    print(total_x)

run(45, 15, 10)
run(45, 30, 20)


# def t(u):
#     return (initial_degree - u) / angular_velocity
#
# def integral_result(u):
#     return t(u)*math.sin(t(u))  + math.cos(t(u))
#
# expected_total_x = -velocity / 15   # calculated from integral for initial_degree 45, and angular vel 15
# expected_total_x = expected_total_x * (-3*math.sqrt(2)) - expected_total_x * -1/angular_velocity * (integral_result(-initial_degree) - integral_result(initial_degree))
#
# print(expected_total_x)


def pi(n=1):
    return math.pi ** n

def integralled(v, t):
    sin = math.sin
    cos = math.cos
    return 12*v/pi(2) * (-pi() * sin(-t*pi()/12 + pi()/4) + 12*cos(-t*pi()/12 + pi()/4))
print(integralled(10, 6) - integralled(10, 0))