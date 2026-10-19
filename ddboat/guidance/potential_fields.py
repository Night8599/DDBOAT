import math
import numpy as np

"""
Point:      V(p) = norm(p - p0) ^ 2 * intensity
            -grad(V(p)) = -2 * (p - p0) * intensity
Line:       V(p) = norm(n^T * (p - p0)) ^ 2 * intensity
            -grad(V(p)) = -2 * n * (n^T * (p - p0)) * intensity
Uniform:    V(p) = -v^T * p * intensity
            -grad(V(p)) = v * intensity

n = normal vector of the line
v = direction vector of the uniform field
p = position vector
p0 = position vector of the point or line
intensity = scaling factor for the potential field (positive for attractive, negative for repulsive)

"""

class PotentialFieldElement:
    def __init__(self, intensity: float, name: str = "PotentialFieldElement"):
        self.intensity = intensity
        self.name = name

    def potential(self, p: np.ndarray) -> float:
        raise NotImplementedError

    def gradient(self, p: np.ndarray) -> np.ndarray:
        raise NotImplementedError
class PointPotentialField(PotentialFieldElement):
    def __init__(self, p0: np.ndarray, intensity: float, name: str = "PointPotentialField"):
        super().__init__(intensity, name)
        self.p0 = p0

    def potential(self, p: np.ndarray) -> float:
        return np.linalg.norm(p - self.p0) ** 2 * self.intensity

    def gradient(self, p: np.ndarray) -> np.ndarray:
        return -2 * (p - self.p0) * self.intensity
class LinePotentialField(PotentialFieldElement):
    def __init__(self, p0: np.ndarray, n: np.ndarray, intensity: float, name: str = "LinePotentialField"):
        super().__init__(intensity, name)
        self.p0 = p0
        self.n = n / np.linalg.norm(n)  # Normalize the normal vector

    def potential(self, p: np.ndarray) -> float:
        return (np.linalg.norm(np.dot(self.n.T, (p - self.p0))) ** 2) * self.intensity

    def gradient(self, p: np.ndarray) -> np.ndarray:
        return -2 * self.n * (np.dot(self.n.T, (p - self.p0))) * self.intensity
class SegmentPotentialField(PotentialFieldElement):
    def __init__(self, p0: np.ndarray, p1: np.ndarray, intensity: float, name: str = "SegmentPotentialField"):
        super().__init__(intensity, name)
        self.p0 = p0
        self.p1 = p1
        self.segment_vector = p1 - p0
        self.segment_length_squared = np.dot(self.segment_vector, self.segment_vector)

    def potential(self, p: np.ndarray) -> float:
        # Project point onto the line defined by the segment
        t = np.dot(p - self.p0, self.segment_vector) / self.segment_length_squared
        t_clamped = np.clip(t, 0.0, 1.0)  # Clamp to the segment
        closest_point = self.p0 + t_clamped * self.segment_vector
        return np.linalg.norm(p - closest_point) ** 2 * self.intensity

    def gradient(self, p: np.ndarray) -> np.ndarray:
        # Project point onto the line defined by the segment
        t = np.dot(p - self.p0, self.segment_vector) / self.segment_length_squared
        t_clamped = np.clip(t, 0.0, 1.0)  # Clamp to the segment
        closest_point = self.p0 + t_clamped * self.segment_vector
        return -2 * (p - closest_point) * self.intensity
class UniformPotentialField(PotentialFieldElement):
    def __init__(self, v: np.ndarray, intensity: float, name: str = "UniformPotentialField"):
        super().__init__(intensity, name)
        self.v = v / np.linalg.norm(v)  # Normalize the vector

    def potential(self, p: np.ndarray) -> float:
        return -np.dot(self.v.T, p) * self.intensity

    def gradient(self, p: np.ndarray) -> np.ndarray:
        return self.v * self.intensity
class PotentialField:
    def __init__(self):
        self.elements = []

    def add_element(self, element: PotentialFieldElement):
        self.elements.append(element)

    def potential(self, p: np.ndarray) -> float:
        total_potential = 0.0
        for element in self.elements:
            total_potential += element.potential(p)
        return total_potential

    def gradient(self, p: np.ndarray) -> np.ndarray:
        total_gradient = np.zeros_like(p)
        for element in self.elements:
            total_gradient += element.gradient(p)
        return total_gradient

    def convert_gradient_to_angle(self, gradient: np.ndarray, degrees: bool = False) -> float:
        """
        Convert the gradient vector to an angle in radians.
        The angle is measured counter-clockwise from the positive x-axis.
        """
        angle = math.atan2(gradient[1], gradient[0])
        if degrees:
            angle = math.degrees(angle)
        return angle
    def convert_gradient_to_esl(self, gradient: np.ndarray) -> float:
        """
        Convert the gradient vector to an ESL (East-South-Left) angle in radians.
        The angle is measured clockwise from the positive x-axis (East).
        """
        angle = math.atan2(-gradient[1], gradient[0])  # Invert y for clockwise measurement
        return angle

if __name__ == "__main__":
    # Example usage
    point_field = PointPotentialField(p0=np.array([1.0, 1.0]), intensity=1.0)
    line_field = LinePotentialField(p0=np.array([0.0, 0.0]), n=np.array([1.0, 1.0]), intensity=1.0)
    segment_field = SegmentPotentialField(p0=np.array([0.0, 0.0]), p1=np.array([1.0, 1.0]), intensity=1.0)
    uniform_field = UniformPotentialField(v=np.array([1.0, 0.0]), intensity=1.0)

    potential_field = PotentialField()
    potential_field.add_element(point_field)
    potential_field.add_element(line_field)
    potential_field.add_element(segment_field)
    potential_field.add_element(uniform_field)
    

    p = np.array([2.0, 2.0])
    print("Total Potential at p:", potential_field.potential(p))
    print("Total Gradient at p:", potential_field.gradient(p))
    gradient = potential_field.gradient(p)
    esl_angle = potential_field.convert_gradient_to_esl(gradient)
    print("ESL Angle of Gradient at p (radians):", esl_angle)
    angle = potential_field.convert_gradient_to_angle(gradient, degrees=True)
    print("Angle of Gradient at p (degrees):", angle)