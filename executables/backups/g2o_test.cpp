#include <cmath>
#include <iostream>


#include "g2o/core/block_solver.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/sparse_optimizer.h"

using namespace std;
using namespace g2o;

int main(){
    SparseOptimizer optimizer;
    typedef BlockSolver<BlockSolverTraits<-1, -1> > SlamBlockSolver;
    typedef LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    auto linearSolver = g2o::make_unique<SlamLinearSolver>();
    linearSolver->setBlockOrdering(false);
    OptimizationAlgorithmGaussNewton* solver =
        new OptimizationAlgorithmGaussNewton(
            g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));

    optimizer.setAlgorithm(solver);
    return 0;
}