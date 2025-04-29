#ifndef NEOHOOKEAN_CONSTRAINT_H
#define NEOHOOKEAN_CONSTRAINT_H

#include <vector>
#include "VectorMath.h"
#include <glm/glm.hpp>

class NeohookeanConstraint {
public:
    // Constructor with indices for the 4 vertices of a tetrahedron
    NeohookeanConstraint(int v1, int v2, int v3, int v4,
                         const std::vector<float>& positions,
                         float youngModulus,
                         float poissonRatio,
                         float alpha);

    // Project the constraint (applies the correction)
    void project(std::vector<float>& positions,
                 const std::vector<float>& invMasses,
                 float& lagrangeMultiplier,
                 float dt) const;

    // Get the indices of the vertices
    const std::vector<int>& getIndices() const { return indices; }

private:
    std::vector<int> indices;    // 4 vertex indices of the tetrahedron
    float V0;                    // Initial volume
    glm::mat3 DmInv;             // Inverse of initial deformation gradient
    float mu;                    // First Lamé parameter
    float lambda;                // Second Lamé parameter
    float alpha;                 // Compliance parameter

    // Helper function to calculate the signed volume of a tetrahedron
    float signedVolume(const std::vector<float>& positions) const;
};

#endif // NEOHOOKEAN_CONSTRAINT_H
