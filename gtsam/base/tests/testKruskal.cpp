/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testKruskal
 * @brief Unit tests for Kruskal's minimum spanning tree algorithm
 * @author Ankur Roy Chowdhury
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/kruskal.h>

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/inference/Ordering.h>

#include <vector>
#include <list>
#include <memory>

gtsam::GaussianFactorGraph makeTestGaussianFactorGraph()
{
    using namespace gtsam;
    using namespace symbol_shorthand;

    GaussianFactorGraph gfg;
    Matrix I = I_2x2;
    Vector2 b(0, 0);
    const SharedDiagonal model = noiseModel::Diagonal::Sigmas(Vector2(0.5, 0.5));
    gfg += JacobianFactor(X(1), I, X(2), I, b, model);
    gfg += JacobianFactor(X(1), I, X(3), I, b, model);
    gfg += JacobianFactor(X(1), I, X(4), I, b, model);
    gfg += JacobianFactor(X(2), I, X(3), I, b, model);
    gfg += JacobianFactor(X(2), I, X(4), I, b, model);
    gfg += JacobianFactor(X(3), I, X(4), I, b, model);

    return gfg;
}

gtsam::NonlinearFactorGraph makeTestNonlinearFactorGraph()
{
    using namespace gtsam;
    using namespace symbol_shorthand;

    NonlinearFactorGraph nfg;
    Matrix I = I_2x2;
    Vector2 b(0, 0);
    const SharedDiagonal model = noiseModel::Diagonal::Sigmas(Vector2(0.5, 0.5));
    nfg += BetweenFactor(X(1), X(2), Rot3(), model);
    nfg += BetweenFactor(X(1), X(3), Rot3(), model);
    nfg += BetweenFactor(X(1), X(4), Rot3(), model);
    nfg += BetweenFactor(X(2), X(3), Rot3(), model);
    nfg += BetweenFactor(X(2), X(4), Rot3(), model);
    nfg += BetweenFactor(X(3), X(4), Rot3(), model);

    return nfg;
}

/* ************************************************************************* */
TEST(kruskal, GaussianFactorGraph)
{
    using namespace gtsam;

    const auto g = makeTestGaussianFactorGraph();

    const FastMap<Key, size_t> forward_ordering = Ordering::Natural(g).invert();
    const auto weights = std::vector<double>(g.size(), 1.0);

    const auto mstEdgeIndices = utils::kruskal(g, forward_ordering, weights);

    // auto PrintMst = [](const auto &graph, const auto &mst_edge_indices)
    // {
    //     std::cout << "MST Edge indices are: \n";
    //     for (const auto &edge : mst_edge_indices)
    //     {
    //         std::cout << edge << " : ";
    //         for (const auto &key : graph[edge]->keys())
    //         {
    //             std::cout << gtsam::DefaultKeyFormatter(gtsam::Symbol(key)) << ", ";
    //         }
    //         std::cout << "\n";
    //     }
    // };

    // PrintMst(g, mstEdgeIndices);

    EXPECT(mstEdgeIndices[0] == 0);
    EXPECT(mstEdgeIndices[1] == 1);
    EXPECT(mstEdgeIndices[2] == 2);
}

/* ************************************************************************* */
TEST(kruskal, NonlinearFactorGraph)
{
    using namespace gtsam;

    const auto g = makeTestNonlinearFactorGraph();

    const FastMap<Key, size_t> forward_ordering = Ordering::Natural(g).invert();
    const auto weights = std::vector<double>(g.size(), 1.0);

    const auto mstEdgeIndices = utils::kruskal(g, forward_ordering, weights);

    // auto PrintMst = [](const auto &graph, const auto &mst_edge_indices)
    // {
    //     std::cout << "MST Edge indices are: \n";
    //     for (const auto &edge : mst_edge_indices)
    //     {
    //         std::cout << edge << " : ";
    //         for (const auto &key : graph[edge]->keys())
    //         {
    //             std::cout << gtsam::DefaultKeyFormatter(gtsam::Symbol(key)) << ", ";
    //         }
    //         std::cout << "\n";
    //     }
    // };

    // PrintMst(g, mstEdgeIndices);

    EXPECT(mstEdgeIndices[0] == 0);
    EXPECT(mstEdgeIndices[1] == 1);
    EXPECT(mstEdgeIndices[2] == 2);
}

/* ************************************************************************* */
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
