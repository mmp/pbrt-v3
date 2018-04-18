import math

# =============================================================================
# Transform callables

# -----------------------------------------------------------------------------
# Normalization into [-1,+1] range
class NormalizeTransform:

    def __init__(self, min_val, max_val):
        self.min_val = min_val
        self.max_val = max_val
    
    def __call__(self, x):
        mid = (self.max_val + self.min_val) / 2.0
        r = self.max_val - mid
        x = x - mid
        x = x / r
        if x < -1.0:
            return -1.0
        elif x > 1.0:
            return 1.0
        else:
            return x

# -----------------------------------------------------------------------------
# Normalization into [0,+1] range

class NormalizePositiveTransform:

    def __init__(self, min_val, max_val):
        self.min_val = min_val
        self.max_val = max_val

    def __call__(self, x):
        d = self.max_val - self.min_val
        x = x - self.min_val
        x = x / d
        if x < 0.0:
            return 0.0
        elif x > 1.0:
            return 1.0
        else:
            return x

class NormalizeInvTransform:

    def __init__(self, max_val):
        self.max_val = max_val
    
    def __call__(self, x):
        return x * self.max_val

# -----------------------------------------------------------------------------

class LogTransform:

    def __init__(self):
        pass
    
    def __call__(self, x):
        if (x + 1.0) <= 0.0:
            return 0.0
        return math.log(x + 1.0)

class LogInvTransform:

    def __init__(self):
        pass
    
    def __call__(self, y):
        if y < 0.0:
            y = 0.0
        return math.exp(y) - 1.0

# -----------------------------------------------------------------------------
class SqrtTransform:

    def __init__(self):
        pass
    
    def __call__(self, x):
        if x < 0.0:
            return 0.0
        return math.sqrt(x)

# -----------------------------------------------------------------------------
class GammaTransform:

    def __init__(self, gm):
        self.exponent = 1.0 / gm
    
    def __call__(self, x):
        if x < 0.0:
            x = 0.0
        return x ** self.exponent

# -----------------------------------------------------------------------------
class Sequence:

    def __init__(self, ts):
        self.ts = ts
    
    def __call__(self, x):
        for t in self.ts:
            x = t(x)
        return x

# -----------------------------------------------------------------------------
# log, normalize+, gamma

class IntensitySequence:

    def __init__(self, max_value, gamma):
        ts = []
        ts.append(LogTransform())
        ts.append(NormalizePositiveTransform(0.0, max_value))
        ts.append(GammaTransform(gamma))
        self.seq = Sequence(ts)
    
    def __call__(self, x):
        return self.seq(x)

class IntensityInvSequence:

    def __init__(self, max_value, gamma):
        ts = []
        ts.append(GammaTransform(1.0 / gamma))
        ts.append(NormalizeInvTransform(max_value))
        ts.append(LogInvTransform())
        self.seq = Sequence(ts)
    
    def __call__(self, x):
        return self.seq(x)

# -----------------------------------------------------------------------------
# sqrt, normalize+, gamma
class DistanceSequence:

    def __init__(self, max_value, gamma):
        ts = []
        ts.append(SqrtTransform())
        ts.append(NormalizePositiveTransform(0.0, max_value))
        ts.append(GammaTransform(gamma))
        self.seq = Sequence(ts)
    
    def __call__(self, x):
        return self.seq(x)