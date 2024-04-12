#include "baseModel2D.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <set>
using namespace std;

/* 
  CSCI 520 Computer Animation and Simulation
  Programming Assignment:
  2D Simulator for Mass spring, Linear FEM and Corotational FEM

  Mianlun Zheng and Jernej Barbic
  University of Southern California
*/

BaseModel2D::BaseModel2D(TriangleMesh2D * mesh_) : mesh(mesh_)
{
  numVertices = mesh_->getNumVertices();
  // store the undeformed positions
  undeformedPositions.resize(2 * numVertices, 0.0);
  for(int i=0; i < numVertices; i++)
  {
    const Vec2d & v = mesh->getRestPosition(i);
    for(int j=0; j<2; j++)
      undeformedPositions[2*i+j] = v[j];
  }

  // ********** Students should implement this **********

  // compute mass for each vertex
  masses.resize(numVertices, 0.0); 
  double density_ = mesh_->getDensity();
  std::vector<Vec3i> triangles = mesh_->getTriangles(); // triangles store the vertex indices for each triangle

  // please calculate the mass for each vertex and store it in the vector masses above
  for (int i = 0; i < mesh_->getNumTriangles(); i++)
  {
      Vec3i verticesIndex = triangles[i];
      Vec2d p1 = Vec2d(undeformedPositions[2 * verticesIndex[0]], undeformedPositions[2 * verticesIndex[0] + 1]);
      Vec2d p2 = Vec2d(undeformedPositions[2 * verticesIndex[1]], undeformedPositions[2 * verticesIndex[1] + 1]);
      Vec2d p3 = Vec2d(undeformedPositions[2 * verticesIndex[2]], undeformedPositions[2 * verticesIndex[2] + 1]);
      Vec2d v1 = p2 - p1;
      Vec2d v2 = p3 - p1;
      double theta = acos(dot(v1, v2) / (len(v1) * len(v2)));
      double area = 0.5 * sin(theta) * len(v1) * len(v2);
      double mass = (area * density_)/3.0;
      masses[verticesIndex[0]] += mass;
      masses[verticesIndex[1]] += mass;
      masses[verticesIndex[2]] += mass;
  }
  // *********************************************

  // create set for edges
  typedef pair<int,int> edge;
  #define SORTED(i,j) ( (i) <= (j) ? make_pair((i),(j)) : make_pair((j),(i)) )
  set<edge> edge_;
  for(int i=0; i<mesh_->getNumTriangles(); i++)
  {
    int v0 = triangles[i][0];
    int v1 = triangles[i][1];
    int v2 = triangles[i][2];
    edge_.insert(SORTED(v0,v1));
    edge_.insert(SORTED(v0,v2));
 
    edge_.insert(SORTED(v1,v0));
    edge_.insert(SORTED(v1,v2));
 
    edge_.insert(SORTED(v2,v0));
    edge_.insert(SORTED(v2,v1));
  }
 
  numEdges = edge_.size(); 
  edges.resize(2 * numEdges, 0);
  
  int count = 0;
  for(set<edge> :: iterator iter = edge_.begin(); iter != edge_.end(); iter++)
  {
    edges[2*count+0] = iter->first;
    edges[2*count+1] = iter->second;
    count++;
  } 
}

BaseModel2D::~BaseModel2D()
{
}

double BaseModel2D::GetTriangleSurfaceArea(Vec2d p0, Vec2d p1, Vec2d p2)
{
  double s = 0.0;
  return s;
}

void BaseModel2D::GenerateMassMatrix(SparseMatrix ** M)
{
    SparseMatrixOutline outline(2 * numVertices); 
    for(int i=0; i<numVertices; i++)
        for(int j=0; j<2; j++)
            outline.AddEntry(2*i+j, 2*i+j, masses[i]); 
    *M = new SparseMatrix(&outline);
}

