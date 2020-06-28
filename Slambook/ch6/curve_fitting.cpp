// g2o - General Graph Optimization
// Copyright (C) 2012 R. Kümmerle
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <Eigen/Dense>
#include <iostream>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/solver.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/stuff/command_args.h>
#include <g2o/stuff/sampler.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

using namespace std;

/**
 * \brief the params, a, b, and lambda for a * exp(-lambda * t) + b
 */
class VertexParams : public g2o::BaseVertex<3, Eigen::Vector3d> 
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexParams() {}

    virtual bool read(std::istream& in) {}
    virtual bool write(std::ostream& out) const {}
    virtual void setToOriginImpl() {
      _estimate(0) = 0;
      _estimate(1) = 0;
      _estimate(2) = 0;
    }
    virtual void oplusImpl(const double *update) {
      _estimate += Eigen::Vector3d(update);
    }
};

class EdgePointOnCurve : public g2o::BaseUnaryEdge<1, Eigen::Vector2d, VertexParams>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual bool read(std::istream& in) {}
    virtual bool write(std::ostream& out) const {}
    void computeError() {
      const VertexParams *params = static_cast<const VertexParams*>(vertex(0));
      const double& a = params->estimate()(0);
      const double& b = params->estimate()(1);
      const double& l = params->estimate()(2);
      double fval = a * exp(-l * measurement()(0)) + b;
      _error(0) = fval - measurement()(1);
    }
};


int main(int argc, char** argv)
{
  int numPoints = 50;
  int maxIterations = 10;
  bool verbose = true;
  std::vector<int> gaugeList;
  
  // generate random data
  double a = 2.0;
  double b = 0.4;
  double l = 0.2;
  Eigen::Vector2d *points = new Eigen::Vector2d[numPoints];
  for (int i = 0; i < numPoints; ++i) {
    double x = g2o::Sampler::uniformRand(0, 10);
    double y = a * exp(-l * x) + b;
    // add Gaussian noise
    y += g2o::Sampler::gaussRand(0,0.02);
    points[i].x() = x;
    points[i].y() = y;
  }

  // Step 1: define block solver type
  typedef g2o::BlockSolver< g2o::BlockSolverTraits<3, 2> >  BlockSolverType;
  // Step 2: select a linear solver
  typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
  // Step 3: select a solver
  g2o::OptimizationAlgorithm* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
  // Step 4: sparse optimizer
  g2o::SparseOptimizer optimizer;     
  optimizer.setAlgorithm( solver );
  optimizer.setVerbose( verbose );

  // Step 5: add vertex and edges to the graph
  // 1. add the parameter vertex
  VertexParams* params = new VertexParams();
  params->setId(0);
  params->setEstimate(Eigen::Vector3d::UnitX());
  optimizer.addVertex(params);
  // 2. add edges
  for (int i = 0; i < numPoints; ++i) {
    EdgePointOnCurve *e = new EdgePointOnCurve();
    e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
    e->setVertex(0, params);
    e->setMeasurement(points[i]);
    optimizer.addEdge(e);
  }

  optimizer.initializeOptimization();
  optimizer.optimize(maxIterations);

  cout << "Target curve" << endl;
  cout << "a * exp(-lambda * x) + b" << endl;
  cout << "Iterative least squares solution" << endl;
  cout << "a      = " << params->estimate()(0) << endl;
  cout << "b      = " << params->estimate()(1) << endl;
  cout << "lambda = " << params->estimate()(2) << endl;
  cout << endl;

  delete[] points;

  return 0;


}
