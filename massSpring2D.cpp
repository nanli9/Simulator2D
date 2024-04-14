#include "massSpring2D.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <set>
#include "macros.h"
using namespace std;
 
MassSpring2D::MassSpring2D(TriangleMesh2D * mesh) : BaseModel2D(mesh)
{  
  double stiffness = mesh->getMassSpringStiffness();
  GenerateMassSpringSystem(numVertices, masses.data(), undeformedPositions, numEdges, edges);
}
 
MassSpring2D::~MassSpring2D()
{
}
 
void MassSpring2D::GenerateMassSpringSystem(int numVertices, const double * masses, const std::vector<double> &restPositions, int numEdges, const std::vector<int> & edges) 
{
  // compute rest lengths of springs
  for(int i=0; i<numEdges; i++)
  {
    int particleA = edges[2*i+0];
    int particleB = edges[2*i+1];
 
    double restDisp[2];
    restDisp[0] = restPositions[particleB*2+0] - restPositions[particleA*2+0]; 
    restDisp[1] = restPositions[particleB*2+1] - restPositions[particleA*2+1];
 
    restLengths.push_back(sqrt(restDisp[0]*restDisp[0] + restDisp[1]*restDisp[1]));
  }
}

void MassSpring2D::ComputeElementEnergyAndForceAndStiffnessMatrix(int eid, const double * u, double * elementEnergy, 
    double * elementInternalForces, double * elementStiffnessMatrix)
{
  double stiffness = mesh->getMassSpringStiffness(); // stiffness of the spring
  int particleA = edges[2*eid+0]; // index of vertex A on the spring
  int particleB = edges[2*eid+1]; // index of vertex B on the spring
  // undeformedPositions, size 2*vertexNumber, stores the rest positions of all verteices
  // u, size 2*vertexNumber, stores the displacements of all verteices

  // ********** Students should implement this **********
  Vec2d rest_Pos_A = Vec2d(undeformedPositions[2 * particleA], undeformedPositions[2 * particleA + 1]);
  Vec2d rest_Pos_B = Vec2d(undeformedPositions[2 * particleB], undeformedPositions[2 * particleB + 1]);
  double R = len(rest_Pos_A - rest_Pos_B);
  Vec2d cur_Pos_A = Vec2d(u[2 * particleA], u[2 * particleA + 1]);
  Vec2d cur_Pos_B = Vec2d(u[2 * particleB], u[2 * particleB + 1]);
  Vec2d L = rest_Pos_A - rest_Pos_B + cur_Pos_A - cur_Pos_B;
  /*if (len(L) > 0.001)
      printf("asd");*/

  if (elementEnergy)
  {
    // please calculate the elastic energy on the spring here
      elementEnergy[0] = 0.5 * stiffness * (len(L) - R) * (len(L) - R);
  }

  if (elementInternalForces)
  {
    // please calculate the the spring forces here
      Vec2d F_A = -stiffness * (len(L) - R) * (L / len(L));
      Vec2d F_B = -F_A;
      elementInternalForces[0] = F_A[0];
      elementInternalForces[1] = F_A[1];
      elementInternalForces[2] = F_B[0];
      elementInternalForces[3] = F_B[1];
  }

  if (elementStiffnessMatrix) 
  {
    // please calculate the stiffness matrix for the spring here
      double K_AA[4];
      double inverse_L = 1.0 / len(L);
      double inverse_L_cubic = 1.0 / (len(L) * len(L) * len(L));
      double diff_x = cur_Pos_A[0] - cur_Pos_B[0];
      double diff_y = cur_Pos_A[1] - cur_Pos_B[1];
      K_AA[0] = stiffness * (1 - R * (inverse_L - inverse_L_cubic * (diff_x * diff_y)));
      K_AA[1] = stiffness * (- R * (-inverse_L_cubic * (diff_x * diff_y)));
      K_AA[2] = stiffness * (- R * (-inverse_L_cubic * (diff_x * diff_y)));
      K_AA[3] = stiffness * (1 - R * (inverse_L - inverse_L_cubic * (diff_x * diff_y)));
      elementStiffnessMatrix[0] = K_AA[0];
      elementStiffnessMatrix[1] = K_AA[1];
      elementStiffnessMatrix[2] = -K_AA[0];
      elementStiffnessMatrix[3] = -K_AA[1];

      elementStiffnessMatrix[4] = K_AA[2];
      elementStiffnessMatrix[5] = K_AA[3];
      elementStiffnessMatrix[6] = -K_AA[2];
      elementStiffnessMatrix[7] = -K_AA[3];

      elementStiffnessMatrix[8] = -K_AA[0];
      elementStiffnessMatrix[9] = -K_AA[1];
      elementStiffnessMatrix[10] = K_AA[0];
      elementStiffnessMatrix[11] = K_AA[1];

      elementStiffnessMatrix[12] = -K_AA[2];
      elementStiffnessMatrix[13] = -K_AA[3];
      elementStiffnessMatrix[14] = K_AA[2];
      elementStiffnessMatrix[15] = K_AA[3];
  }

  // ***********************************************

  postProcess(elementInternalForces, elementStiffnessMatrix);
}

const int * MassSpring2D::getVertexIndices(int stencilID)
{
  return edges.data() + stencilID*2;
}

void MassSpring2D::postProcess(double * elementInternalForces, double * elementStiffnessMatrix)
{
  if(elementInternalForces)
  {
    for(int i=0; i<4; i++)
      elementInternalForces[i] = -elementInternalForces[i];
  }
  if(elementStiffnessMatrix)
  {
    for(int i=0; i<16; i++)
      elementStiffnessMatrix[i] = -elementStiffnessMatrix[i];
  }
}
