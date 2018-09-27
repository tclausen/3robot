class Pid:
    def __init__(self, p, d, i):
        self.p = p
        self.d = d
        self.i = i
        self.totalE = 0
        self.ep = 0
        
    def f(self, e, dt):
        # P term
        p = -self.p * e
        # D term
        d = self.i * (self.ep-e)/dt
        # I term
        self.totalE = self.totalE + e*dt
        i = self.i * self.totalE
        
        self.ep = e
        correction = int(p + d + i)
        
        return correction
