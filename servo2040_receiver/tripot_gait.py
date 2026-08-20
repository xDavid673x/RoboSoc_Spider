from math import cos, pi, radians, sin
import time
try:
    import servo_control
    is_simulate = False
except ImportError:
    is_simulate = True
class Tripot_gait:
    def __init__(self, beta_ang = pi/4, use_hardware_batch = None):
        self.use_hardware_batch = (
            not is_simulate
            if use_hardware_batch is None
            else bool(use_hardware_batch)
        )
        self.legs_ROT_dict = {
            'legi' : self.rotation_matrix(0),
            'legl' : self.rotation_matrix(pi),
            'legj' : self.rotation_matrix(beta_ang),
            'legm' : self.rotation_matrix(pi + beta_ang),
            'legk' : self.rotation_matrix(pi/2 + beta_ang),
            'legn' : self.rotation_matrix(-beta_ang)
        }
        a = pi if not is_simulate else 0
        b = 1 if not is_simulate else -1
        self.anti_beta_dict = {
            'legi' : a,
            'legl' : a + pi*b,
            'legj' : a + beta_ang*b,
            'legm' : a + (pi + beta_ang)*b,
            'legk' : a + (pi/2 + beta_ang)*b,
            'legn' : a + (-beta_ang)*b
        }
        self.tripod_A = {'legi', 'legk', 'legm'}
        self.tripod_B = {'legj', 'legl', 'legn'}

        self.tripods = [self.tripod_A, self.tripod_B]
        self.walk_t = 0
        self.walk_tripod_idx = 0
        self.turn_t = 0
        self.turn_tripod_idx = 0
        self.transition_steps = 12
        self.walk_blend_remaining = 0
        self.walk_blend_total = 0
        self.walk_start_targets = {}
        self.turn_blend_remaining = 0
        self.turn_blend_total = 0
        self.turn_start_targets = {}

    def bezier_curve(self, p1, p2, p3, i, steps, duration = 0.0):
        t = i / steps
        te = 0.5 * (1 - cos(pi * t))  # easing

        y = (1 - te)**2 * p1[0] + 2 * (1 - te) * te * p2[0] + te**2 * p3[0]
        z = (1 - te)**2 * p1[1] + 2 * (1 - te) * te * p2[1] + te**2 * p3[1]
        if duration > 0:
            time.sleep(duration / steps)
        return y, z
    
    def rotation_matrix(self, beta):
        ROT = [
            [cos(beta), -sin(beta), 0],
            [sin(beta), cos(beta),  0],
            [0,         0,          1]
        ]
        return ROT

    def _resolve_body_height(self, body_height, S):
        if body_height is None:
            return S
        return body_height

    def calculate_gait_targets(self, legs, swing_legs, p1, p3, body_height, A, num_of_steps, t, angle, x_pos, duration = 0.0):
        """Calculate IK targets for all legs at time step t."""
        targets = {}
        S = body_height

        # swing & stance curves
        p2_up = [0, S + 2*A]
        p2_down = [0, S]

        y_up, z_up = self.bezier_curve(p1, p2_up, p3, t, num_of_steps, duration)
        y_down, z_down = self.bezier_curve(p1, p2_down, p3, t, num_of_steps, duration)

        for leg in legs:
            R_leg = self.rotation_matrix(self.anti_beta_dict[leg.name])

            if leg.name in swing_legs:
                y, z = y_up, z_up
                sign = +1
            else:
                y, z = y_down, z_down
                sign = -1

            target = [
                sign * R_leg[0][1] * y,
                sign * R_leg[1][1] * y,
                sign * R_leg[2][1] * y,
            ]

            # body rotation
            R_body = self.rotation_matrix(radians(angle))
            target_rot = [
                x_pos   + R_body[0][0]*target[0] + R_body[0][1]*target[1] + R_body[0][2]*target[2],
                0       + R_body[1][0]*target[0] + R_body[1][1]*target[1] + R_body[1][2]*target[2],
                z       + R_body[2][0]*target[0] + R_body[2][1]*target[1] + R_body[2][2]*target[2],
            ]

            targets[leg.name] = target_rot
        return targets

    def _advance_phase(self, step, t, tripod_idx):
        t += 1
        if t > step:
            t = 0
            tripod_idx = (tripod_idx + 1) % 2
        return t, tripod_idx

    def _snapshot_tip_targets(self, legs):
        current = {}
        for leg in legs:
            tip = leg.forwardKinematics()[3]
            current[leg.name] = [tip[0], tip[1], tip[2]]
        return current

    def _smooth_alpha(self, i, total):
        if total <= 0:
            return 1.0
        t = i / total
        return 0.5 * (1 - cos(pi * t))

    def _blend_targets(self, start_targets, target_targets, alpha):
        blended = {}
        for leg_name, tgt in target_targets.items():
            s = start_targets.get(leg_name, tgt)
            blended[leg_name] = [
                s[0] + (tgt[0] - s[0]) * alpha,
                s[1] + (tgt[1] - s[1]) * alpha,
                s[2] + (tgt[2] - s[2]) * alpha,
            ]
        return blended

    def _apply_targets(self, legs, targets):
        angles_by_leg = []
        for leg in legs:
            angles_by_leg.append((leg, leg.calculate_inverse_angles(targets[leg.name])))
        if self.use_hardware_batch:
            servo_control.begin_batch()
        try:
            for leg, angles in angles_by_leg:
                leg.set_angles(angles)
        finally:
            if self.use_hardware_batch:
                servo_control.end_batch()
        for leg, _ in angles_by_leg:
            leg.forwardKinematics()

    def reset_walk_phase(self, transition_steps = None):
        self.walk_t = 0
        self.walk_tripod_idx = 0
        steps = self.transition_steps if transition_steps is None else int(transition_steps)
        self.walk_blend_total = max(0, steps)
        self.walk_blend_remaining = self.walk_blend_total
        self.walk_start_targets = {}

    def reset_turn_phase(self, transition_steps = None):
        self.turn_t = 0
        self.turn_tripod_idx = 0
        steps = self.transition_steps if transition_steps is None else int(transition_steps)
        self.turn_blend_total = max(0, steps)
        self.turn_blend_remaining = self.turn_blend_total
        self.turn_start_targets = {}

    def walk_step(self, legs, angle = 0, T = 120, body_height = None, S = -100, A = 20, step = 50, xpos = 150):
        body_height = self._resolve_body_height(body_height, S)
        p1 = [-T / 2, body_height]
        p3 = [T / 2, body_height]
        swing_tripod = self.tripods[self.walk_tripod_idx]
        targets = self.calculate_gait_targets(
            legs, swing_tripod,
            p1, p3, body_height, A,
            step, self.walk_t,
            angle,
            xpos,
            0.0
        )

        if self.walk_blend_remaining > 0:
            if not self.walk_start_targets:
                self.walk_start_targets = self._snapshot_tip_targets(legs)
            done = self.walk_blend_total - self.walk_blend_remaining + 1
            alpha = self._smooth_alpha(done, self.walk_blend_total)
            targets = self._blend_targets(self.walk_start_targets, targets, alpha)
            self.walk_blend_remaining -= 1

        self._apply_targets(legs, targets)
        self.walk_t, self.walk_tripod_idx = self._advance_phase(step, self.walk_t, self.walk_tripod_idx)

    def turn_step(self, legs, turn_ratio = 1.0, max_angle = 40, T = 100, body_height = None, S = -100, A = 28, step = 50, xpos = 150):
        body_height = self._resolve_body_height(body_height, S)
        angle = max_angle * turn_ratio
        p1 = [-T / 2, body_height]
        p3 = [T / 2, body_height]
        swing_tripod = self.tripods[self.turn_tripod_idx]
        targets = self.calculate_gait_targets(
            legs, swing_tripod,
            p1, p3, body_height, A,
            step, self.turn_t,
            angle,
            xpos,
            0.0
        )

        if self.turn_blend_remaining > 0:
            if not self.turn_start_targets:
                self.turn_start_targets = self._snapshot_tip_targets(legs)
            done = self.turn_blend_total - self.turn_blend_remaining + 1
            alpha = self._smooth_alpha(done, self.turn_blend_total)
            targets = self._blend_targets(self.turn_start_targets, targets, alpha)
            self.turn_blend_remaining -= 1

        self._apply_targets(legs, targets)
        self.turn_t, self.turn_tripod_idx = self._advance_phase(step, self.turn_t, self.turn_tripod_idx)
    
    def movement(self, legs, angle = 0, T = 120, body_height = None, S = -100, A = 20, step = 50, xpos = 150):
        body_height = self._resolve_body_height(body_height, S)
        self.reset_walk_phase()
        while True:
            self.walk_step(legs, angle, T, body_height, S, A, step, xpos)
            time.sleep(0.002)

    def turning(self, legs, angle = 40, body_height = None, S = -100, A = 50, xpos = 150):
        body_height = self._resolve_body_height(body_height, S)
        self.reset_turn_phase()
        while True:
            self.turn_step(legs, 1.0 if angle >= 0 else -1.0, abs(angle), 100, body_height, S, A, 50, xpos)
            time.sleep(0.002)
            
