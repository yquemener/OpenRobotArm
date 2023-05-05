import math


def d2r(b):
    return b / 180.0 * math.pi


def r2d(a):
    return round(a * 180 / math.pi)

def clamp(num, min_value=0., max_value=1.):
   return max(min(num, max_value), min_value)

class Link:
    def __init__(self, length, angle_low_limit, angle_high_limit):
        self._length = length
        self._angle = self._angle_low = angle_low_limit
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
        self.current_phi = 0

    def init_braccio(self):
        self.attach(
            Link(0, d2r(0.0), d2r(180.0)),
            Link(130, d2r(0.0), d2r(162.0)),
            Link(125, d2r(0.0), d2r(188.0)),
            Link(65, d2r(0.0), d2r(184.0)))

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
        base = math.atan2(y, x) + math.pi/2

        result = self.solve_semipolar_iv(base, _r, z - self.L0.get_length(), phi)

        if not result:
            return False

        base, shoulder, elbow, wrist = result

        # If there is a solution, return the angles
        return base, shoulder, elbow, wrist

    def solve_semipolar(self, alpha, y, z, phi=0):
        _r = y
        base = alpha

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

    def solve_semipolar_iv(self, alpha, y, z, phi=0):
        _r = y
        base = alpha

        # shoulder to wrist distance
        s2w = math.sqrt(z ** 2 + _r ** 2)
        elbow_angle = math.acos(clamp((self.L1._length**2 + self.L2._length**2-s2w**2)/(2*self.L1._length*self.L2._length)
                                      , min_value=-1))
        elbow_angle -= math.pi / 2
        elbow_angle = max(min(elbow_angle, self.L2._angle_high), self.L2._angle_low)


        sigma = math.atan2(z, _r)

        v_angle = math.acos(clamp((s2w ** 2 + self.L1._length ** 2 - self.L2._length ** 2) / (2 * self.L1._length * s2w),
                                  min_value=-1))

        shoulder_angle = sigma + v_angle
        shoulder_angle = max(min(shoulder_angle, self.L1._angle_high), self.L1._angle_low)

        wrist_angle = 3*math.pi/2 - shoulder_angle - elbow_angle - phi
        return base, shoulder_angle, elbow_angle, wrist_angle

    def _solve_fixed_phi(self, x, y, phi):
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
        ret = self._solve_fixed_phi(x, y, self.current_phi)

        if ret != False:
            return ret

        degree_step = 0.0174533  # Approximately 1 degree in radians
        double_pi = 2 * math.pi

        for i in range(int(double_pi*2/degree_step)):
            phi = -double_pi + i * degree_step
            ret = self._solve_fixed_phi(x, y, phi)
            if ret != False:
                self.current_phi = phi
                return ret

            phi = i*degree_step
            ret = self._solve_fixed_phi(x, y, phi)
            if ret != False:
                self.current_phi = phi
                return ret
        return False
