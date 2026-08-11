#!/usr/bin/env python3

import numpy as np


class CalibrationResult:
    """Container for a completed magnetometer calibration."""

    def __init__(
        self,
        hard_iron,
        soft_iron,
        fit_error,
        mean_radius,
        std_radius,
    ):
        self.hard_iron = np.asarray(hard_iron, dtype=np.float64)
        self.soft_iron = np.asarray(soft_iron, dtype=np.float64)
        self.fit_error = float(fit_error)
        self.mean_radius = float(mean_radius)
        self.std_radius = float(std_radius)

    def as_dict(self):
        return {
            "hard_iron": self.hard_iron,
            "soft_iron": self.soft_iron,
            "fit_error": self.fit_error,
            "mean_radius": self.mean_radius,
            "std_radius": self.std_radius,
        }


class MagnetometerCalibrator:
    """
    Numerical magnetometer calibration using ellipsoid fitting.

    Calibrated measurement:

        m_calibrated = soft_iron @ (m_raw - hard_iron)
    """

    def __init__(self, min_samples=1000):
        if min_samples < 10:
            raise ValueError("min_samples must be at least 10.")

        self.min_samples = int(min_samples)
        self._samples = []
        self._result = None

    def reset(self):
        """Clear all samples and previous calibration."""
        self._samples.clear()
        self._result = None

    def add_sample(self, x, y, z):
        """Add one magnetometer sample."""
        sample = np.asarray(
            [x, y, z],
            dtype=np.float64,
        )

        if not np.all(np.isfinite(sample)):
            raise ValueError(
                "Magnetometer sample contains NaN or infinity."
            )

        self._samples.append(sample)

    @property
    def sample_count(self):
        return len(self._samples)

    @property
    def ready(self):
        return self.sample_count >= self.min_samples

    @property
    def result(self):
        return self._result

    def calibrate(self):
        """
        Perform ellipsoid calibration.

        Returns:
            CalibrationResult
        """

        if not self.ready:
            raise ValueError(
                f"Need at least {self.min_samples} samples, "
                f"got {self.sample_count}."
            )

        data = np.asarray(
            self._samples,
            dtype=np.float64,
        )

        if not np.all(np.isfinite(data)):
            raise ValueError(
                "Calibration data contains NaN or infinity."
            )

        hard_iron, soft_iron = self._fit_ellipsoid(data)

        corrected = data - hard_iron

        raw_radius = np.linalg.norm(
            corrected,
            axis=1,
        )

        target_radius = np.mean(raw_radius)

        soft_iron *= target_radius

        calibrated = (
            data - hard_iron
        ) @ soft_iron.T

        magnitudes = np.linalg.norm(
            calibrated,
            axis=1,
        )

        mean_radius = float(
            np.mean(magnitudes)
        )

        std_radius = float(
            np.std(magnitudes)
        )

        if mean_radius <= 0.0:
            raise ValueError(
                "Invalid calibration: mean field magnitude is zero."
            )

        fit_error = std_radius / mean_radius

        self._result = CalibrationResult(
            hard_iron=hard_iron,
            soft_iron=soft_iron,
            fit_error=fit_error,
            mean_radius=mean_radius,
            std_radius=std_radius,
        )

        return self._result

    @staticmethod
    def _fit_ellipsoid(data):
        """
        Fit:

            x^T A x + b^T x + c = 0

        and extract:

            hard_iron = ellipsoid center
            soft_iron = transformation to a sphere
        """

        x = data[:, 0]
        y = data[:, 1]
        z = data[:, 2]

        design = np.column_stack([
            x * x,
            y * y,
            z * z,
            2.0 * x * y,
            2.0 * x * z,
            2.0 * y * z,
            2.0 * x,
            2.0 * y,
            2.0 * z,
            np.ones_like(x),
        ])

        _, singular_values, vh = np.linalg.svd(
            design,
            full_matrices=False,
        )

        if len(singular_values) < 10:
            raise ValueError(
                "Insufficient information for ellipsoid fitting."
            )

        if singular_values[-1] <= 0.0:
            raise ValueError(
                "Degenerate ellipsoid fit."
            )

        coefficients = vh[-1].copy()

        if abs(coefficients[-1]) < 1e-12:
            raise ValueError(
                "Degenerate ellipsoid fit."
            )

        coefficients /= -coefficients[-1]

        a, b, c, d, e, f, g, h, i, constant = coefficients

        matrix = np.array(
            [
                [a, d, e],
                [d, b, f],
                [e, f, c],
            ],
            dtype=np.float64,
        )

        linear = np.array(
            [g, h, i],
            dtype=np.float64,
        )

        matrix = 0.5 * (
            matrix + matrix.T
        )

        try:
            hard_iron = -0.5 * np.linalg.solve(
                matrix,
                linear,
            )
        except np.linalg.LinAlgError as exc:
            raise ValueError(
                "Degenerate ellipsoid fit: "
                "quadratic matrix is singular."
            ) from exc

        k = (
            hard_iron @ matrix @ hard_iron
            - constant
        )

        if not np.isfinite(k) or k <= 0.0:
            raise ValueError(
                "Invalid ellipsoid fit. "
                "Collect samples over more orientations."
            )

        eigenvalues, eigenvectors = np.linalg.eigh(
            matrix
        )

        if np.any(eigenvalues <= 0.0):
            raise ValueError(
                "Invalid ellipsoid fit: "
                "ellipsoid is not positive definite."
            )

        transform = (eigenvectors @ np.diag(np.sqrt(eigenvalues / k))@ eigenvectors.T)

        return hard_iron, transform

    def apply_calibration(self, reading):
        """
        Apply the most recent calibration to one reading.

        Args:
            reading: [x, y, z]

        Returns:
            numpy.ndarray
        """

        if self._result is None:
            raise RuntimeError(
                "Calibration has not been performed."
            )

        reading = np.asarray(
            reading,
            dtype=np.float64,
        )

        if reading.shape != (3,):
            raise ValueError(
                "Magnetometer reading must contain exactly 3 values."
            )

        return self._result.soft_iron @ (
            reading - self._result.hard_iron
        )

    def evaluate(self):
        """Return calibration quality metrics."""

        if self._result is None:
            raise RuntimeError(
                "Calibration has not been performed."
            )

        return {
            "fit_error": self._result.fit_error,
            "mean_radius": self._result.mean_radius,
            "std_radius": self._result.std_radius,
            "sphericity": 1.0 - self._result.fit_error,
        }