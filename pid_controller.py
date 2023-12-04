class PIDController:
    def __init__(self, kP, kI, kD):
        self.kP = kP
        self.kI = kI
        self.kD = kD

        self.e_of_t = {}
        self.current_time = 0;

    def calculate(self, error):
        self.e_of_t[self.current_time] = error

        output = sum([
            self.kP * self.calculate_proportional(),
            self.kI * self.calculate_integral(),
            self.kD * self.calculate_derivative()
        ])

        self.current_time += 1
        return output

    def calculate_proportional(self):
        return self.e_of_t[self.current_time]

    def calculate_integral(self):
        integral = 0
        for t in range(self.current_time):
            integral += self.e_of_t[t]

        return integral

    def calculate_derivative(self):
        if self.current_time == 0:
            return 0 # GD REFERENCE
        
        return self.e_of_t[self.current_time] - self.e_of_t[self.current_time + 1]