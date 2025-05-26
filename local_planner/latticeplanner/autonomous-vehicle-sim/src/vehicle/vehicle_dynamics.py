class VehicleDynamics:
    def __init__(self, mass, drag_coefficient):
        self.mass = mass
        self.drag_coefficient = drag_coefficient
        self.position = 0.0
        self.velocity = 0.0

    def calculate_acceleration(self, thrust):
        drag_force = self.drag_coefficient * self.velocity ** 2
        net_force = thrust - drag_force
        return net_force / self.mass

    def update_position(self, time_step, thrust):
        acceleration = self.calculate_acceleration(thrust)
        self.velocity += acceleration * time_step
        self.position += self.velocity * time_step

    def get_state(self):
        return self.position, self.velocity