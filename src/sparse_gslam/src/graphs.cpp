#include "graphs.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"

void setup_lm_opt(g2o::SparseOptimizer& opt) {
    using SlamBlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<-1, 2>>;
    using SlamLinearSolver = g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType>;
    opt.setAlgorithm(new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<SlamBlockSolver>(g2o::make_unique<SlamLinearSolver>())));
    opt.setVerbose(false);
    opt.setComputeBatchStatistics(false);
}

void setup_pose_opt(g2o::SparseOptimizer& opt) {
    using SlamBlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<3, 3>>;
    using SlamLinearSolver = g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType>;
    opt.setAlgorithm(new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<SlamBlockSolver>(g2o::make_unique<SlamLinearSolver>())));
    opt.setVerbose(false);
    opt.setComputeBatchStatistics(false);
}

LandmarkGraph::LandmarkGraph() {
    setup_lm_opt(opt);
}
LandmarkGraph::~LandmarkGraph() {
    delete opt.algorithm();
}

PoseGraph::PoseGraph() {
    setup_pose_opt(opt);
}
PoseGraph::~PoseGraph() {
    delete opt.algorithm();
}