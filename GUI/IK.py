import math

class Link:
    def __init__(self):
        self._length = 0.0
        self._angle_low = 0.0
        self._angle_high = 0.0
        self._angle = 0.0

    def init(self, length, angle_low_limit, angle_high_limit):
        self._length = length
        self._angle_low = angle_low_limit
        self._angle_high = angle_high_limit

    def in_range(self, angle):
        return self._angle_low <= angle <= self._angle_high

    def get_length(self):
        return self._length

    def get_angle(self):
        return self._angle

    def set_angle(self, angle):
        self._angle = angle

class InverseIK:
    def __init__(self):
        self.L0 = None  # Link 0: Shoulder
        self.L1 = None  # Link 1: Upper arm
        self.L2 = None  # Link 2: Forearm
        self.L3 = None  # Link 3: Hand
        self.current_phi = None

    def attach(self, shoulder, upperarm, forearm, hand):
        self.L0 = shoulder
        self.L1 = upperarm
        self.L2 = forearm
        self.L3 = hand

    def cosrule(self, opposite, adjacent1, adjacent2):
        delta = 2 * adjacent1 * adjacent2
        if delta == 0:
            return False
        cos = (adjacent1 ** 2 + adjacent2 ** 2 - opposite ** 2) / delta
        if cos > 1 or cos < -1:
            return False
        angle = math.acos(cos)
        return angle

    def solve(self, x, y, z, phi=0):
        # Solve the angle of the base
        _r = math.sqrt(x * x + y * y)
        base = math.atan2(y, x)

        # Check the range of the base
        if not self.L0.in_range(base):
            # If not in range, flip the angle
            base += math.pi if base < 0 else -math.pi
            _r *= -1
            if phi != 0:
                phi = math.pi - phi

        # Solve XY (RZ) for the arm plane
        if phi == 0:
            result = self._solve_free_phi(_r, z - self.L0.get_length())
        else:
            result = self._solve_fixed_phi(_r, z - self.L0.get_length(), phi)

        if not result:
            return False

        shoulder, elbow, wrist = result

        # If there is a solution, return the angles
        return base, shoulder, elbow, wrist

    def _solve_fixed_phi(self, x, y, phi, shoulder, elbow, wrist):
        # Adjust coordinate system for base as ground plane
        _r = math.sqrt(x * x + y * y)
        _theta = math.atan2(y, x)
        _x = _r * math.cos(_theta - (math.pi / 2))
        _y = _r * math.sin(_theta - (math.pi / 2))
        _phi = phi - (math.pi / 2)

        # Find the coordinate for the wrist
        xw = _x - self.L3.get_length() * math.cos(_phi)
        yw = _y - self.L3.get_length() * math.sin(_phi)

        # Get polar system
        alpha = math.atan2(yw, xw)
        R = math.sqrt(xw * xw + yw * yw)

        # Calculate the inner angle of the shoulder
        beta = self.cosrule(self.L2.get_length(), R, self.L1.get_length())

        if beta is False:
            return False

        # Calculate the inner angle of the elbow
        gamma = self.cosrule(R, self.L1.get_length(), self.L2.get_length())

        if gamma is False:
            return False

        # Solve the angles of the arm
        shoulder = alpha - beta
        elbow = math.pi - gamma
        wrist = _phi - shoulder - elbow

        # Check the range of each hinge
        if (not self.L1.in_range(shoulder) or
                not self.L2.in_range(elbow) or
                not self.L3.in_range(wrist)):
            # If not in range, solve for the second solution
            shoulder += 2 * beta
            elbow *= -1
            wrist = _phi - shoulder - elbow

            # Check the range for the second solution
            if (not self.L1.in_range(shoulder) or
                    not self.L2.in_range(elbow) or
                    not self.L3.in_range(wrist)):
                return False
        return shoulder, elbow, wrist

    def _solve_free_phi(self, x, y):
        success, shoulder, elbow, wrist = self._solve_fixed_phi(x, y, self.current_phi)

        if success:
            return shoulder, elbow, wrist

        degree_step = 0.0174533  # Approximately 1 degree in radians
        double_pi = 2 * math.pi

        for phi in range(-double_pi, double_pi, degree_step):
            success, shoulder, elbow, wrist = self._solve_fixed_phi(x, y, phi)

            if success:
                self.current_phi = phi
                return shoulder, elbow, wrist
        return False
