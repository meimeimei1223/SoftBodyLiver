#include "Neohookeanconstraint.h"
#include <algorithm>
#include <cmath>
#include <iostream>

NeohookeanConstraint::NeohookeanConstraint(
    int v1, int v2, int v3, int v4,
    const std::vector<float>& positions,
    float youngModulus,
    float poissonRatio,
    float alpha)
    : indices({v1, v2, v3, v4}), V0(0.0f), alpha(alpha)
{
    // Extract positions of the 4 vertices
    glm::vec3 p1(positions[v1*3], positions[v1*3+1], positions[v1*3+2]);
    glm::vec3 p2(positions[v2*3], positions[v2*3+1], positions[v2*3+2]);
    glm::vec3 p3(positions[v3*3], positions[v3*3+1], positions[v3*3+2]);
    glm::vec3 p4(positions[v4*3], positions[v4*3+1], positions[v4*3+2]);

    // Calculate the deformation matrix Dm
    glm::mat3 Dm;
    Dm[0] = p1 - p4;
    Dm[1] = p2 - p4;
    Dm[2] = p3 - p4;

    // Calculate initial volume
    V0 = (1.0f / 6.0f) * glm::determinant(Dm);

    // Calculate inverse of Dm
    DmInv = glm::inverse(Dm);

    // Calculate Lamé parameters from Young's modulus and Poisson's ratio
    mu = youngModulus / (2.0f * (1.0f + poissonRatio));
    lambda = (youngModulus * poissonRatio) / ((1.0f + poissonRatio) * (1.0f - 2.0f * poissonRatio));
}

void NeohookeanConstraint::project(
    std::vector<float>& positions,
    const std::vector<float>& invMasses,
    float& lagrangeMultiplier,
    float dt) const
{
    const int v1 = indices[0];
    const int v2 = indices[1];
    const int v3 = indices[2];
    const int v4 = indices[3];

    // Extract positions
    glm::vec3 p1(positions[v1*3], positions[v1*3+1], positions[v1*3+2]);
    glm::vec3 p2(positions[v2*3], positions[v2*3+1], positions[v2*3+2]);
    glm::vec3 p3(positions[v3*3], positions[v3*3+1], positions[v3*3+2]);
    glm::vec3 p4(positions[v4*3], positions[v4*3+1], positions[v4*3+2]);

    // Extract inverse masses
    float w1 = invMasses[v1];
    float w2 = invMasses[v2];
    float w3 = invMasses[v3];
    float w4 = invMasses[v4];

    // Check signed volume for inversion detection
    float Vsigned = signedVolume(positions);
    bool isVPositive = Vsigned >= 0.0f;
    bool isV0Positive = V0 >= 0.0f;
    bool isTetInverted = (isVPositive && !isV0Positive) || (!isVPositive && isV0Positive);

    // Calculate current deformation matrix Ds
    glm::mat3 Ds;
    Ds[0] = p1 - p4;
    Ds[1] = p2 - p4;
    Ds[2] = p3 - p4;

    // Calculate deformation gradient F
    glm::mat3 F = Ds * DmInv;
    const glm::mat3 I = glm::mat3(1.0f); // Identity matrix

    const float epsilon = 1e-20f;

    glm::mat3 Piola;
    float psi = 0.0f;

    // Handle inverted or non-inverted tetrahedra
    if (isTetInverted) {
        // SVD decomposition (simplified approximation for this implementation)
        // Full SVD implementation would be more complex
        glm::vec3 singularValues;
        glm::mat3 U, V;

        // This is a simplified SVD decomposition for demonstration
        // In practice, use a proper SVD library
        // For now, we'll just handle it with a simplified approach

        // Calculate F^T * F
        glm::mat3 F2 = glm::transpose(F) * F;

        // Simple estimation of singular values (not accurate but serves as placeholder)
        singularValues.x = std::sqrt(F2[0][0]);
        singularValues.y = std::sqrt(F2[1][1]);
        singularValues.z = std::sqrt(F2[2][2]);

        // Find smallest singular value
        int smallestIdx = 0;
        if (singularValues.y < singularValues.x && singularValues.y < singularValues.z)
            smallestIdx = 1;
        if (singularValues.z < singularValues.x && singularValues.z < singularValues.y)
            smallestIdx = 2;

        // Modify the smallest singular value (inversion handling)
        singularValues[smallestIdx] = -singularValues[smallestIdx];

        // Clamp minimum singular value
        const float minSingularValue = 0.577f;
        singularValues.x = std::min(singularValues.x, minSingularValue);
        singularValues.y = std::min(singularValues.y, minSingularValue);
        singularValues.z = std::min(singularValues.z, minSingularValue);

        // We'll use a regularized approach instead of full SVD
        glm::mat3 Fprime = F;
        glm::mat3 Finv = glm::inverse(Fprime);
        glm::mat3 FinvT = glm::transpose(Finv);

        float I1 = F2[0][0] + F2[1][1] + F2[2][2]; // Trace
        float J = glm::determinant(Fprime);
        J = std::max(J, 0.05f); // Prevent collapse

        float logJ = std::log(J);

        // Neohookean energy
        psi = 0.5f * mu * (I1 - 3.0f) - mu * logJ + 0.5f * lambda * logJ * logJ;

        // Neohookean stress (Piola-Kirchhoff)
        Piola = mu * (Fprime - mu * FinvT) + lambda * logJ * FinvT;
    }
    else {
        // Standard case (no inversion)
        glm::mat3 F2 = glm::transpose(F) * F;
        glm::mat3 Finv = glm::inverse(F);
        glm::mat3 FinvT = glm::transpose(Finv);

        float I1 = F2[0][0] + F2[1][1] + F2[2][2]; // Trace
        float J = glm::determinant(F);
        J = std::max(J, 0.05f); // Prevent collapse

        float logJ = std::log(J);

        // Neohookean energy
        psi = 0.5f * mu * (I1 - 3.0f) - mu * logJ + 0.5f * lambda * logJ * logJ;

        // Neohookean stress (Piola-Kirchhoff)
        Piola = mu * (F - mu * FinvT) + lambda * logJ * FinvT;
    }

    // Compute forces (negative gradient of elastic potential)
    float V0Abs = std::abs(V0);
    glm::mat3 H = -V0Abs * Piola * glm::transpose(DmInv);

    glm::vec3 f1 = H[0];
    glm::vec3 f2 = H[1];
    glm::vec3 f3 = H[2];
    glm::vec3 f4 = -(f1 + f2 + f3);

    // Weighted sum of gradient norms
    float weightedSumOfGradients =
        w1 * glm::dot(f1, f1) +
        w2 * glm::dot(f2, f2) +
        w3 * glm::dot(f3, f3) +
        w4 * glm::dot(f4, f4);

    if (weightedSumOfGradients < epsilon)
        return;

    // XPBD constraint solving
    float C = V0Abs * psi;
    float alphaTilde = alpha / (dt * dt);
    float deltaLagrange = -(C + alphaTilde * lagrangeMultiplier) / (weightedSumOfGradients + alphaTilde);

    lagrangeMultiplier += deltaLagrange;

    // Apply position corrections
    if (w1 > 0.0f) {
        positions[v1*3]   += w1 * -f1.x * deltaLagrange;
        positions[v1*3+1] += w1 * -f1.y * deltaLagrange;
        positions[v1*3+2] += w1 * -f1.z * deltaLagrange;
    }

    if (w2 > 0.0f) {
        positions[v2*3]   += w2 * -f2.x * deltaLagrange;
        positions[v2*3+1] += w2 * -f2.y * deltaLagrange;
        positions[v2*3+2] += w2 * -f2.z * deltaLagrange;
    }

    if (w3 > 0.0f) {
        positions[v3*3]   += w3 * -f3.x * deltaLagrange;
        positions[v3*3+1] += w3 * -f3.y * deltaLagrange;
        positions[v3*3+2] += w3 * -f3.z * deltaLagrange;
    }

    if (w4 > 0.0f) {
        positions[v4*3]   += w4 * -f4.x * deltaLagrange;
        positions[v4*3+1] += w4 * -f4.y * deltaLagrange;
        positions[v4*3+2] += w4 * -f4.z * deltaLagrange;
    }
}

float NeohookeanConstraint::signedVolume(const std::vector<float>& positions) const {
    const int v1 = indices[0];
    const int v2 = indices[1];
    const int v3 = indices[2];
    const int v4 = indices[3];

    glm::vec3 p1(positions[v1*3], positions[v1*3+1], positions[v1*3+2]);
    glm::vec3 p2(positions[v2*3], positions[v2*3+1], positions[v2*3+2]);
    glm::vec3 p3(positions[v3*3], positions[v3*3+1], positions[v3*3+2]);
    glm::vec3 p4(positions[v4*3], positions[v4*3+1], positions[v4*3+2]);

    glm::mat3 Ds;
    Ds[0] = p1 - p4;
    Ds[1] = p2 - p4;
    Ds[2] = p3 - p4;

    return (1.0f / 6.0f) * glm::determinant(Ds);
}
