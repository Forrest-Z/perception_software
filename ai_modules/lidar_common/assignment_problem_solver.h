/*
 *Hungarian algorithm with task allocation procedures
 */
#ifndef ASSIGNMENT_PROBLEM_SOLVER_H
#define ASSIGNMENT_PROBLEM_SOLVER_H
#include <vector>
#include <limits>
#include <time.h>

namespace perception {
namespace lidar {

class AssignmentProblemSolver
{
public:

    AssignmentProblemSolver();
    ~AssignmentProblemSolver();
    double Solve(const std::vector< std::vector<double> >& DistMatrix, std::vector<int>& Assignment);

private:
    // --------------------------------------------------------------------------
    // Computes the optimal assignment (minimum overall costs) using Munkres algorithm.
    // --------------------------------------------------------------------------
    void assignmentoptimal(int *assignment, double *cost, double *distMatrix, int nOfRows, int nOfColumns);
    void buildassignmentvector(int *assignment, bool *starMatrix, int nOfRows, int nOfColumns);
    void computeassignmentcost(int *assignment, double *cost, double *distMatrix, int nOfRows);
    void step2a(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
    void step2b(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
    void step3(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
    void step4(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col);
    void step5(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
    // --------------------------------------------------------------------------
    // Computes a suboptimal solution. Good for cases with many forbidden assignments.
    // --------------------------------------------------------------------------
    void assignmentsuboptimal1(int *assignment, double *cost, double *distMatrixIn, int nOfRows, int nOfColumns);
    //贪心算法
    void assignmentsuboptimal2(int *assignment, double *cost, double *distMatrixIn, int nOfRows, int nOfColumns);
};

}  // namespace lidar
}  // namespace perception

#endif // ASSIGNMENT_PROBLEM_SOLVER_H
