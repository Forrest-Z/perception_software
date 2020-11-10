#include "lidar_common/calculate_cost_matrix.h"

namespace perception {
namespace lidar {

CalculateCostMatrix::CalculateCostMatrix()
{

}

CalculateCostMatrix::~CalculateCostMatrix()
{

}

void CalculateCostMatrix::calculateEuclideanDistance(const std::vector<base::ObjectPtr>& detections,
                                                     const std::vector<base::ObjectPtr>& trackerObjects,
                                                     std::vector< std::vector<double> > &costMatrix)
{
    float dist = 0;
    for (size_t i = 0; i < trackerObjects.size(); i++)
    {
        for (size_t j = 0; j < detections.size(); j++)
        {
            cv::Point2f point1(trackerObjects[i]->center[0],
                               trackerObjects[i]->center[1]);
            cv::Point2f point2(detections[j]->center[0], detections[j]->center[1]);
            cv::Point2f diff = (point1 - point2);
            dist = std::sqrt(diff.x*diff.x + diff.y*diff.y);
            costMatrix[i][j] = static_cast<double>(dist);
        }
    }
}

void CalculateCostMatrix::calculateMahalanobisDistance(const std::vector<base::ObjectPtr>& detections,
                                                       const std::vector<base::ObjectPtr>& trackerObjects,
                                                       std::vector< std::vector<double> > &costMatrix)
{
    cv::Mat samples(trackerObjects.size(), 4, CV_32FC1);
    cv::Mat covar;
    cv::Mat invCovar;
    cv::Mat mean;
    for (size_t i = 0; i < trackerObjects.size(); i++)
    {
        float* data = samples.ptr<float>(i);
        data[0] = trackerObjects[i]->center[0];
        data[1] = trackerObjects[i]->center[1];
        data[2] = trackerObjects[i]->size[0];
        data[3] = trackerObjects[i]->size[1];
    }
    cv::calcCovarMatrix(samples, covar, mean, CV_COVAR_NORMAL|CV_COVAR_ROWS, CV_32F);
    covar = covar / (samples.rows - 1);
    cv::invert(covar, invCovar);
    for (size_t i = 0; i < trackerObjects.size(); i++)
    {
        base::ObjectPtr trackObject = trackerObjects[i];
        cv::Mat vec1 = (cv::Mat_<float>(1, 4) << trackObject->center[0], trackObject->center[1],
                        trackObject->size[0], trackObject->size[1]);
        for (size_t j = 0; j < detections.size(); j++)
        {
            cv::Mat vec2 = (cv::Mat_<float>(1, 4) << detections[j]->center[0], detections[j]->center[1],
                    detections[j]->size[0], detections[j]->size[1]);
            costMatrix[i][j] = cv::Mahalanobis(vec1, vec2, invCovar);
        }
    }
}

}  // namespace lidar
}  // namespace perception
