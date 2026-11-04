"""Independent pitch must not inherit roll/yaw calibration conservatism."""
import unittest
import numpy as np
from nearspace.model6 import VehicleParams
from nearspace.torque_gap_authority import (
    AuthorityConfig, authority_axis_scales, axis_hull, build_boxes,
    calibrate_scale, natural_moment_v, surface_matrix,
)


class PitchAuthorityScaleTest(unittest.TestCase):
    def test_full_pitch_interval_at_32_degrees(self):
        veh = VehicleParams()
        alpha = np.deg2rad(32)
        unit_nat = natural_moment_v(1, alpha, 0, np.zeros(3), 100, veh)
        Q = .55 / (unit_nat[1] + axis_hull(1, veh)[1])
        cond = dict(rho=np.array([2*Q/100**2]), V=np.array([100.]),
                    V_ned=np.array([[100*np.cos(alpha), 0, 100*np.sin(alpha)]]),
                    q_ref=np.array([[1., 0, 0, 0]]), omega_ref=np.zeros((1, 3)))
        box = build_boxes([0], cond, veh, AuthorityConfig(scale=.774, calibrate=False))
        nat = natural_moment_v(Q, alpha, 0, np.zeros(3), 100, veh)
        half = axis_hull(Q, veh)
        self.assertAlmostEqual(box['tau_max'][0, 1], .55, places=10)
        self.assertLess(nat[1] + .774*half[1], 0)
        np.testing.assert_allclose((box['tau_max']-box['tau_min'])[0]/2,
                                   half*np.array([.774, 1., .774]))
        np.testing.assert_allclose((box['tau_max']+box['tau_min'])[0]/2, nat)

    def test_calibration_metadata_and_decoupled_axes(self):
        veh = VehicleParams()
        scale, diagnostics = calibrate_scale(veh, [1., 10.], n_samples=40)
        self.assertEqual(diagnostics[-1]['axis_scales'][1], 1.)
        np.testing.assert_allclose(authority_axis_scales(veh, scale), [scale, 1, scale])
        independent = veh.copy(Cl_dr=0., Cn_da=0.)
        np.testing.assert_array_equal(authority_axis_scales(independent, .2), np.ones(3))
        self.assertEqual(surface_matrix(1, veh)[1, 1], 0.)


if __name__ == '__main__':
    unittest.main()
