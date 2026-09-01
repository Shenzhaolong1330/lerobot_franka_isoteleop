import unittest

import torch

try:
    from scripts.server.sign_invariant_cartesian_impedance import (
        SignInvariantCartesianSpacePDFast,
    )
except ModuleNotFoundError as error:
    if error.name != "torchcontrol":
        raise
    SignInvariantCartesianSpacePDFast = None


@unittest.skipIf(
    SignInvariantCartesianSpacePDFast is None,
    "torchcontrol is installed only in the polymetis-local runtime",
)
class QuaternionSignPolicyTest(unittest.TestCase):
    @staticmethod
    def make_module() -> SignInvariantCartesianSpacePDFast:
        return SignInvariantCartesianSpacePDFast(
            torch.ones(6),
            torch.zeros(6),
            torch.eye(3),
            torch.ones(6),
        )

    def test_q_and_negative_q_produce_identical_wrench(self) -> None:
        module = self.make_module()
        scripted = torch.jit.script(module)
        position = torch.zeros(3)
        twist = torch.zeros(6)
        angle = torch.tensor(0.04)
        desired = torch.stack(
            [
                torch.tensor(0.0),
                torch.tensor(0.0),
                torch.sin(angle / 2.0),
                torch.cos(angle / 2.0),
            ]
        )
        current = torch.tensor([0.0, 0.0, 0.0, 1.0])

        wrench_positive = scripted(
            position,
            current,
            twist,
            position,
            desired,
            twist,
        )
        wrench_negative = scripted(
            position,
            current,
            twist,
            position,
            -desired,
            twist,
        )

        torch.testing.assert_close(wrench_positive, wrench_negative)
        self.assertGreater(float(wrench_positive[5]), 0.0)

    def test_equal_physical_orientation_has_zero_orientation_wrench(self) -> None:
        module = torch.jit.script(
            self.make_module()
        )
        position = torch.zeros(3)
        twist = torch.zeros(6)
        current = torch.tensor([0.2, -0.3, 0.1, 0.92736185])
        current = current / torch.linalg.vector_norm(current)

        wrench = module(
            position,
            current,
            twist,
            position,
            -current,
            twist,
        )

        torch.testing.assert_close(wrench[3:], torch.zeros(3), atol=1e-7, rtol=0.0)

    def test_error_is_clipped_in_selection_frame(self) -> None:
        selection_to_base = torch.tensor(
            [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
        )
        module = torch.jit.script(
            SignInvariantCartesianSpacePDFast(
                torch.ones(6),
                torch.zeros(6),
                selection_to_base,
                torch.tensor([0.01, 0.02, 0.03, 1.0, 1.0, 1.0]),
            )
        )
        current_quaternion = torch.tensor([0.0, 0.0, 0.0, 1.0])
        wrench = module(
            torch.zeros(3),
            current_quaternion,
            torch.zeros(6),
            torch.tensor([0.0, 0.1, 0.0]),
            current_quaternion,
            torch.zeros(6),
        )

        # Selection X is base Y, and is capped at 0.01 m.
        torch.testing.assert_close(
            wrench[:3], torch.tensor([0.0, 0.01, 0.0]), atol=1e-7, rtol=0.0
        )


if __name__ == "__main__":
    unittest.main()
