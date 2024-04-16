#include "corotationalLinearFEM2D.h"
#include "mat3d.h"

/* 
  CSCI 520 Computer Animation and Simulation
  Programming Assignment:
  2D Simulator for Mass spring, Linear FEM and Corotational FEM

  Mianlun Zheng and Jernej Barbic
  University of Southern California
*/
double* InverseMat2x2(double* m)
{
    double determintInversem = 1.0 / (m[0] * m[3] - m[1] * m[2]);
    double* inverse = new double[4];
    inverse[0] = determintInversem * m[3];
    inverse[1] = -determintInversem * m[1];
    inverse[2] = -determintInversem * m[2];
    inverse[3] = determintInversem * m[0];
    return inverse;
}
double* MultiplicationMat2x2(double* m1,double* m2)
{
    double* result = new double[4];
    result[0] = m1[0] * m2[0] + m1[1] * m2[2];
    result[1] = m1[0] * m2[1] + m1[1] * m2[3];
    result[2] = m1[2] * m2[0] + m1[3] * m2[2];
    result[3] = m1[2] * m2[1] + m1[3] * m2[3];
    return result;
}

CorotationalLinearFEM2D::CorotationalLinearFEM2D(TriangleMesh2D * mesh_): LinearFEM2D(mesh_)
{
}

CorotationalLinearFEM2D::~CorotationalLinearFEM2D()
{
}

void CorotationalLinearFEM2D::polarDecomposition2D(const double * F, double * R, double * S)
{
  Vec2d temp(F[0]+F[3], F[2]-F[1]);
  Vec2d temp2 = norm(temp);

  double R_[4] = {temp2[0], -temp2[1], temp2[1], temp2[0]};
  memcpy(R, R_, sizeof(double) * 4);

  double S_[4] = {R_[0]*F[0]+R_[2]*F[2], R_[0]*F[1]+R_[2]*F[3], R_[1]*F[0]+R_[3]*F[2], R_[1]*F[1]+R_[3]*F[3]};
  memcpy(S, S_, sizeof(double) * 4);
}

void CorotationalLinearFEM2D::inverse2x2(const double * A, double * AInv)
{ 
}

void CorotationalLinearFEM2D::ComputeElementEnergyAndForceAndStiffnessMatrix(int el, const double * u, double * elementEnergy, 
    double * elementInternalForces, double * elementStiffnessMatrix)
{
  const int numElementVertices = 3; // each triangle has 3 vertices
  const int numElementDOFs = numElementVertices * 2; // in total 3 *2 dimensions for one triangle
  const int * vtxIndex = mesh->getVertexIndices(el);// vertex index for the triangle el
  // undeformedPositions, size 2*vertexNumber, stores the rest positions of all verteices
  // u, size 2*vertexNumber, stores the displacements of all verteices

  // ********** Students should implement this **********

  // please calculate the deformation gradient matrix F here
  // F = Ds * DmInv
  double Fe[4] = {1.0, 0.0, 0.0, 1.0};
  double Dm[4] = { undeformedPositions[2 * vtxIndex[0]] - undeformedPositions[2 * vtxIndex[2]], 
                   undeformedPositions[2 * vtxIndex[1]] - undeformedPositions[2 * vtxIndex[2]],
                   undeformedPositions[2 * vtxIndex[0] + 1] - undeformedPositions[2 * vtxIndex[2] + 1],
                   undeformedPositions[2 * vtxIndex[1] + 1] - undeformedPositions[2 * vtxIndex[2] + 1]};
  double Ds[4] = { Dm[0] + u[2 * vtxIndex[0]] - u[2 * vtxIndex[2]],
                   Dm[1] + u[2 * vtxIndex[1]] - u[2 * vtxIndex[2]],
                   Dm[2] + u[2 * vtxIndex[0] + 1] - u[2 * vtxIndex[2] + 1],
                   Dm[3] + u[2 * vtxIndex[1] + 1] - u[2 * vtxIndex[2] + 1]};
  double determintInverseDm = 1.0 / (Dm[0] * Dm[3] - Dm[1] * Dm[2]);
  double DmInv[4] = { determintInverseDm * Dm[3], -determintInverseDm * Dm[1],
                      -determintInverseDm* Dm[2], determintInverseDm * Dm[0]};

  Fe[0] = Ds[0] * DmInv[0] + Ds[1] * DmInv[2];
  Fe[1] = Ds[0] * DmInv[1] + Ds[1] * DmInv[3];
  Fe[2] = Ds[2] * DmInv[0] + Ds[3] * DmInv[2];
  Fe[3] = Ds[2] * DmInv[1] + Ds[3] * DmInv[3];



  // polar decomposition of F is provided here. no need to implement it
  double Re[4]; // rotation (row-major)
  double Se[4]; // symmetric (row-major)
  polarDecomposition2D(Fe, Re, Se);
  // now the rotation Re is computed and stored in Re[4] array
  double* Re_inverse = InverseMat2x2(Re);
  double RE[36];
  double RE_INVERSE[36];
  /*for (int i = 0; i < 36; i++)
  {
      RE[i] = 0;
      RE_INVERSE[i] = 0;
  }*/
  for (int i = 0; i < 3; i++)
  {
      for (int j = 0; j < 3; j++)
      {
          if (i == j)
          {
              RE[3 * 4 * i + 2 * j] = Re[0];
              RE[3 * 4 * i + 2 * j + 1] = Re[1];
              RE[3 * 4 * i + 2 * j + 6] = Re[2];
              RE[3 * 4 * i + 2 * j + 7] = Re[3];

              RE_INVERSE[3 * 4 * i + 2 * j] = Re_inverse[0];
              RE_INVERSE[3 * 4 * i + 2 * j + 1] = Re_inverse[1];
              RE_INVERSE[3 * 4 * i + 2 * j + 6] = Re_inverse[2];
              RE_INVERSE[3 * 4 * i + 2 * j + 7] = Re_inverse[3];

          }
          else
          {
              RE[3 * 4 * i + 2 * j] = 0;
              RE[3 * 4 * i + 2 * j + 1] = 0;
              RE[3 * 4 * i + 2 * j + 6] = 0;
              RE[3 * 4 * i + 2 * j + 7] = 0;

              RE_INVERSE[3 * 4 * i + 2 * j] = 0;
              RE_INVERSE[3 * 4 * i + 2 * j + 1] = 0;
              RE_INVERSE[3 * 4 * i + 2 * j + 6] = 0;
              RE_INVERSE[3 * 4 * i + 2 * j + 7] = 0;
          }
      }
  }
  double a[36];
      for (int i = 0; i < 6; i++)
      {
          for (int j = 0; j < 6; j++)
          {
              a[6 * i + j] = 0;
              for (int k = 0; k < 6; k++)
                  a[6 * i + j] += RE[6 * i + k] * RE_INVERSE[6 * k + j];



          }
      }

  if(elementStiffnessMatrix)
  {
    // please calculate the warped stiffness matrix K' here
    // the unwarped stiffness matrix for element el is stored in KElementUndeformed[el]
      double tmp[36];
      for (int i = 0; i < 6; i++)
      {
          for (int j = 0; j < 6; j++)
          {
              tmp[6 * i + j] = 0;
              for (int k = 0; k < 6; k++)
                  tmp[6 * i + j] += RE[6 * i + k] * KElementUndeformed[el][6 * k + j];



          }
      }
      for (int i = 0; i < 6; i++)
      {
          for (int j = 0; j < 6; j++)
          {
              elementStiffnessMatrix[6 * i + j] = 0;
              for (int k = 0; k < 6; k++)
                  elementStiffnessMatrix[6 * i + j] += tmp[6 * i + k] * RE_INVERSE[6 * k + j];

          }
      }


  }
  double RE_KE[36];
  double RE_INVERSE_PE_MINUS_QE[6];
  for (int i = 0; i < 6; i++)
  {
      double sum = 0;
      for (int j = 0; j < 6; j++)
      {
          RE_KE[6 * i + j] = 0;
          for (int k = 0; k < 6; k++)
              RE_KE[6 * i + j] += RE[6 * i + k] * KElementUndeformed[el][6 * k + j];
      }
      for (int j = 0; j < 3; j++)
      {
          sum += RE_INVERSE[6 * i + 2 * j] * (undeformedPositions[2 * vtxIndex[j]] + u[2 * vtxIndex[j]]);
          sum += RE_INVERSE[6 * i + 2 * j + 1] * (undeformedPositions[2 * vtxIndex[j] + 1] + u[2 * vtxIndex[j] + 1]);
      }
      sum -= undeformedPositions[2 * vtxIndex[i / 2] + i % 2];
      RE_INVERSE_PE_MINUS_QE[i] = sum;
  }
  if(elementEnergy)
  {
    // please calculate the elastic energy here
      double tmp[6];
      for (int i = 0; i < 6; i++)
      {
          double sum = 0;
          for (int j = 0; j < 6; j++)
          {
              sum += KElementUndeformed[el][6 * i + j] * RE_INVERSE_PE_MINUS_QE[j];
          }
          tmp[i] = sum;
      }
      for (int i = 0; i < 6; i++)
      {
          elementEnergy[0] += tmp[i] * RE_INVERSE_PE_MINUS_QE[i];
      }
      elementEnergy[0] /= 2;
  }

  if(elementInternalForces)
  {
    // please calculate the internal force here
      
      for (int i = 0; i < 6; i++)
      {
          double sum = 0;
          for (int j = 0; j < 6; j++)
          {
              sum += RE_KE[6 * i + j] * RE_INVERSE_PE_MINUS_QE[j];
          }
          elementInternalForces[i] = sum;
      }
  }

  // ******************************************
}

// compute RK = R * K and RKRT = R * K * R^T (block-wise)
// input: K, R
// output: RK, RKRT
void CorotationalLinearFEM2D::WarpMatrix(double * K, double * R, double * RK, double * RKRT)
{
}

